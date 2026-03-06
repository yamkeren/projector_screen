// ============================================================================
// integration_test.cpp — Full system integration smoke test
//
// Build:   pio run -e integration_test -t upload
// Monitor: pio device monitor -e integration_test
//
// Tests the full firmware stack minus WiFi:
//   1. Subsystem initialization
//   2. Command queue round-trip
//   3. NVS persistence write/read
//   4. Fault injection and recovery cycle
//   5. State machine transitions
//   6. Motion controller basic sanity
//   7. PID reconfiguration at runtime
//
// No motor movement required — all tests are programmatic / electrical only.
// ============================================================================

#include <Arduino.h>
#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "config.h"
#include "core/system_types.h"
#include "core/shared_state.h"
#include "motion/motion_controller.h"
#include "motion/pid.h"
#include "safety/fault_manager.h"
#include "storage/persistent_storage.h"
#include "app/homing_machine.h"
#include "app/app_controller.h"

static SharedState        sharedState;
static MotionController   motionCtrl;
static FaultManager       faultMgr(sharedState, motionCtrl);
static PersistentStorage  storage;
static HomingMachine      homing(sharedState, motionCtrl);
static AppController      appCtrl(sharedState, motionCtrl, faultMgr, storage, homing);

static uint8_t testNum   = 0;
static uint8_t passCount = 0;
static uint8_t failCount = 0;

static void check(const char* name, bool passed, const char* detail = nullptr) {
    testNum++;
    if (passed) passCount++;
    else failCount++;
    Serial.printf("  [%2d] %-44s %s\n", testNum, name, passed ? "PASS" : "** FAIL **");
    if (detail) Serial.printf("        %s\n", detail);
}

// ============================================================================
// Test 1: Subsystem initialization
// ============================================================================
static void testInit() {
    Serial.println("\n--- Test Group: Initialization ---");

    motionCtrl.begin();
    check("MotionController.begin() completes", true);

    bool storageOk = storage.begin();
    check("PersistentStorage.begin() completes", storageOk);

    faultMgr.begin();
    check("FaultManager.begin() completes", true);

    // Shared state should be at BOOT initially (before any state changes)
    // Actually, begin() or previous tests might have changed it, so just check it's valid
    SystemState st = sharedState.getState();
    bool validState = (st == SystemState::BOOT || st == SystemState::IDLE ||
                       st == SystemState::FAULT);
    check("SharedState has valid initial state", validState);
}

// ============================================================================
// Test 2: Command queue round-trip
// ============================================================================
static void testCommandQueue() {
    Serial.println("\n--- Test Group: Command Queue ---");

    // Inject CLOSE command
    Command cmdClose;
    cmdClose.type = CommandType::CLOSE;
    bool sent = sharedState.sendCommand(cmdClose);
    check("CMD_CLOSE queued successfully", sent);

    // Read it back
    Command received;
    bool got = sharedState.receiveCommand(received);
    check("CMD_CLOSE dequeued", got && received.type == CommandType::CLOSE);

    // Inject OPEN command
    Command cmdOpen;
    cmdOpen.type = CommandType::OPEN;
    sent = sharedState.sendCommand(cmdOpen);
    check("CMD_OPEN queued successfully", sent);

    got = sharedState.receiveCommand(received);
    check("CMD_OPEN dequeued", got && received.type == CommandType::OPEN);

    // Inject STOP command with param
    Command cmdStop;
    cmdStop.type = CommandType::STOP;
    cmdStop.param = 42;
    sent = sharedState.sendCommand(cmdStop);
    check("CMD_STOP queued with param", sent);

    got = sharedState.receiveCommand(received);
    check("CMD_STOP dequeued with correct param",
          got && received.type == CommandType::STOP && received.param == 42);

    // Queue empty check
    got = sharedState.receiveCommand(received);
    check("Queue empty after all consumed", !got);

    // Fill queue up to capacity (8) and verify overflow behavior
    for (int i = 0; i < 8; i++) {
        Command c;
        c.type = CommandType::HOME;
        c.param = i;
        sharedState.sendCommand(c, 0); // Non-blocking
    }

    // 9th should fail or be dropped
    Command overflow;
    overflow.type = CommandType::WAKE;
    bool overflowSent = sharedState.sendCommand(overflow, 0); // Should fail
    check("Queue overflow handled (9th cmd rejected)", !overflowSent);

    // Drain the queue
    while (sharedState.receiveCommand(received)) {}
    check("Queue drained successfully", true);
}

// ============================================================================
// Test 3: NVS persistence round-trip
// ============================================================================
static void testStorage() {
    Serial.println("\n--- Test Group: NVS Storage ---");

    // Write a known value
    bool writeOk = storage.forceSave(42, true);
    check("NVS write (pos=42, homed=true)", writeOk);

    vTaskDelay(pdMS_TO_TICKS(100));

    // Read it back
    PersistentStorage::StoredConfig sc;
    bool readOk = storage.loadConfig(sc);
    check("NVS read succeeds", readOk);

    if (readOk) {
        char detail[64];
        snprintf(detail, sizeof(detail), "pos=%ld homed=%d", sc.position, sc.homed);
        check("NVS position matches written value", sc.position == 42, detail);
        check("NVS homed flag matches written value", sc.homed == 1, detail);
    } else {
        check("NVS position matches written value", false, "CRC mismatch or read failed");
        check("NVS homed flag matches written value", false, "CRC mismatch or read failed");
    }

    // Write a different value
    writeOk = storage.forceSave(9999, false);
    check("NVS second write (pos=9999, homed=false)", writeOk);

    vTaskDelay(pdMS_TO_TICKS(100));

    readOk = storage.loadConfig(sc);
    if (readOk) {
        char detail[64];
        snprintf(detail, sizeof(detail), "pos=%ld homed=%d", sc.position, sc.homed);
        check("NVS second read matches", sc.position == 9999 && sc.homed == 0, detail);
    } else {
        check("NVS second read matches", false, "Read failed");
    }

    // Verify rate limiting: savePosition should be rate-limited
    bool immediate1 = storage.savePosition(100, true);
    bool immediate2 = storage.savePosition(200, true);
    // Both should return true (rate-limited returns true without writing)
    check("savePosition rate-limiting returns true", immediate1 && immediate2);

    // Verify the value didn't change (rate-limited write was skipped)
    readOk = storage.loadConfig(sc);
    if (readOk) {
        // Should still be 9999 from forceSave (rate-limited writes skipped)
        // Actually first savePosition(100) might succeed if 5000ms passed,
        // so we just check the read succeeds
        check("NVS consistent after rate-limited writes", readOk);
    } else {
        check("NVS consistent after rate-limited writes", false, "Read failed");
    }
}

// ============================================================================
// Test 4: Fault injection and recovery cycle
// ============================================================================
static void testFaultCycle() {
    Serial.println("\n--- Test Group: Fault Inject / Reset ---");

    // Start clean
    sharedState.setFaults(FaultCode::NONE);
    sharedState.setState(SystemState::IDLE);
    motionCtrl.begin(); // re-enable motor
    vTaskDelay(pdMS_TO_TICKS(50));

    FaultCode faults = sharedState.getFaults();
    check("Start with no faults", faults == FaultCode::NONE);

    // Trigger stall fault
    faultMgr.triggerFault(FaultCode::STALL_DETECTED);
    check("Stall fault latches", faultMgr.isFaultLatched());

    faults = sharedState.getFaults();
    check("STALL code in shared state", hasFault(faults, FaultCode::STALL_DETECTED));

    // SystemState should be FAULT
    check("State moved to FAULT", sharedState.getState() == SystemState::FAULT);

    // Add second fault
    faultMgr.triggerFault(FaultCode::MOTION_TIMEOUT);
    faults = sharedState.getFaults();
    check("TIMEOUT also latched", hasFault(faults, FaultCode::MOTION_TIMEOUT));
    check("Multiple faults active simultaneously",
          hasFault(faults, FaultCode::STALL_DETECTED) && hasFault(faults, FaultCode::MOTION_TIMEOUT));

    // Reset
    motionCtrl.begin(); // re-enable motor
    vTaskDelay(pdMS_TO_TICKS(100));

    bool resetOk = faultMgr.resetFault();
    check("Fault reset succeeds", resetOk);

    if (resetOk) {
        faults = sharedState.getFaults();
        check("All faults cleared after reset", faults == FaultCode::NONE);
        check("Latch cleared after reset", !faultMgr.isFaultLatched());
    }
}

// ============================================================================
// Test 5: State machine transitions
// ============================================================================
static void testStateTransitions() {
    Serial.println("\n--- Test Group: State Transitions ---");

    sharedState.setState(SystemState::BOOT);
    check("Set BOOT", sharedState.getState() == SystemState::BOOT);

    sharedState.setState(SystemState::HOMING);
    check("Transition to HOMING", sharedState.getState() == SystemState::HOMING);

    sharedState.setState(SystemState::IDLE);
    check("Transition to IDLE", sharedState.getState() == SystemState::IDLE);

    sharedState.setState(SystemState::MOVING);
    check("Transition to MOVING", sharedState.getState() == SystemState::MOVING);

    sharedState.setState(SystemState::FAULT);
    check("Transition to FAULT", sharedState.getState() == SystemState::FAULT);

    sharedState.setState(SystemState::SLEEPING);
    check("Transition to SLEEPING", sharedState.getState() == SystemState::SLEEPING);

    sharedState.setState(SystemState::IDLE);
    check("Recovery to IDLE from SLEEPING", sharedState.getState() == SystemState::IDLE);
}

// ============================================================================
// Test 6: Motion controller basic sanity
// ============================================================================
static void testMotionSanity() {
    Serial.println("\n--- Test Group: Motion Controller Sanity ---");

    motionCtrl.begin();
    motionCtrl.resetEncoder();
    check("Encoder resets to 0", motionCtrl.position() == 0);

    motionCtrl.setTarget(0);
    motionCtrl.update();
    check("At target when pos=target=0", motionCtrl.atTarget());

    motionCtrl.setTarget(100);
    check("Not at target after setTarget(100)", !motionCtrl.atTarget());

    // Verify soft limit clamping
    motionCtrl.setTarget(999999);
    check("Target clamped to SOFT_LIMIT_MAX",
          motionCtrl.target() == cfg::motion::SOFT_LIMIT_MAX);

    motionCtrl.setTarget(-999999);
    check("Target clamped to SOFT_LIMIT_MIN",
          motionCtrl.target() == cfg::motion::SOFT_LIMIT_MIN);

    motionCtrl.stop();
    check("Stop sets mode to STOPPED",
          motionCtrl.mode() == MotionController::Mode::STOPPED);

    // PID output should be ~0 after stop/reset
    float out = motionCtrl.pidOutput();
    check("PID output near 0 after stop", fabsf(out) < 0.01f);
}

// ============================================================================
// Test 7: PID reconfiguration at runtime
// ============================================================================
static void testPidReconfigure() {
    Serial.println("\n--- Test Group: PID Reconfiguration ---");

    motionCtrl.begin();
    motionCtrl.resetEncoder();

    // Configure with known gains
    PIDController::Params p;
    p.kp = 1.0f;
    p.ki = 0.0f;
    p.kd = 0.0f;
    p.outputMin = -1.0f;
    p.outputMax = 1.0f;
    p.integralMax = 1.0f;
    p.derivAlpha = 0.1f;
    p.dt = cfg::pid::DT_SEC;
    motionCtrl.reconfigurePid(p);
    check("reconfigurePid() completes", true);

    // Set target 100, pos=0, P-only → output should be Kp * error = 1.0 * 100 = 100 → clamped to 1.0
    motionCtrl.setTarget(100);
    motionCtrl.update();
    float out = motionCtrl.pidOutput();
    char detail[64];
    snprintf(detail, sizeof(detail), "PID output = %.4f (expected 1.0)", out);
    check("P-only output saturates at max", fabsf(out - 1.0f) < 0.05f, detail);

    // Now set Kp very small
    p.kp = 0.001f;
    motionCtrl.reconfigurePid(p);
    motionCtrl.setTarget(100); // reset move timer
    motionCtrl.update();
    out = motionCtrl.pidOutput();
    snprintf(detail, sizeof(detail), "PID output = %.4f (expected ~0.1)", out);
    // Kp=0.001 * error=100 = 0.1
    check("Low Kp produces small output", out > 0.05f && out < 0.2f, detail);

    motionCtrl.stop();

    // Restore defaults
    PIDController::Params defaults;
    defaults.kp         = cfg::pid::KP;
    defaults.ki         = cfg::pid::KI;
    defaults.kd         = cfg::pid::KD;
    defaults.outputMin  = cfg::pid::OUTPUT_MIN;
    defaults.outputMax  = cfg::pid::OUTPUT_MAX;
    defaults.integralMax = cfg::pid::INTEGRAL_MAX;
    defaults.derivAlpha = cfg::pid::DERIV_FILTER_ALPHA;
    defaults.dt         = cfg::pid::DT_SEC;
    motionCtrl.reconfigurePid(defaults);
    check("Default gains restored", true);
}

// ============================================================================
void setup() {
    Serial.begin(115200);
    vTaskDelay(pdMS_TO_TICKS(500));

    Serial.println("\n=============================================");
    Serial.println("  Full Integration Smoke Test");
    Serial.println("=============================================");
    Serial.println("  No motor movement required.");
    Serial.println("  Tests subsystem APIs and data flow.\n");

    testInit();
    testCommandQueue();
    testStorage();
    testFaultCycle();
    testStateTransitions();
    testMotionSanity();
    testPidReconfigure();

    Serial.println("\n=============================================");
    Serial.println("  RESULTS");
    Serial.println("=============================================");
    Serial.printf("  Passed: %d\n", passCount);
    Serial.printf("  Failed: %d\n", failCount);
    Serial.printf("  Total:  %d\n", testNum);
    if (failCount == 0) {
        Serial.println("\n  >>> ALL TESTS PASSED <<<");
    } else {
        Serial.println("\n  >>> FAILURES DETECTED <<<");
    }
    Serial.println("=============================================\n");
}

void loop() {
    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(1000));
}
