// ============================================================================
// safety_test.cpp — Fault injection and safety subsystem validation
//
// Build:   pio run -e safety_test -t upload
// Monitor: pio device monitor -e safety_test
//
// Tests:
//   1. INA226 current sensor communication and stability
//   2. Limit switch reading and noise check
//   3. Fault latching via triggerFault() and reset
//   4. Stall detection (drive motor into obstruction / block motor shaft)
//
// Prerequisites:
//   - INA226 connected on I2C (SDA=21, SCL=22, addr=0x40)
//   - Limit switch on pin 32
//   - For stall test: motor shaft should be BLOCKED or motor disconnected
// ============================================================================

#include <Arduino.h>
#include <esp_task_wdt.h>
#include "config.h"
#include "core/system_types.h"
#include "core/shared_state.h"
#include "motion/motion_controller.h"
#include "safety/current_sensor.h"
#include "safety/fault_manager.h"

static SharedState      sharedState;
static MotionController motionCtrl;
static CurrentSensor    currentSensor;
static FaultManager     faultMgr(sharedState, motionCtrl, currentSensor);

static uint8_t testNumber = 0;
static uint8_t passCount  = 0;
static uint8_t failCount  = 0;

static void check(const char* name, bool passed, const char* detail = nullptr) {
    testNumber++;
    if (passed) passCount++;
    else failCount++;

    Serial.printf("  [%2d] %-44s %s\n", testNumber, name, passed ? "PASS" : "** FAIL **");
    if (detail) {
        Serial.printf("        %s\n", detail);
    }
}

// ============================================================================
// Test 1: INA226 communication and current reading
// ============================================================================
static void testCurrentSensor() {
    Serial.println("\n--- Test: INA226 Current Sensor ---");

    bool initOk = currentSensor.begin();
    check("INA226 initializes successfully", initOk);

    if (!initOk) {
        check("INA226 returns valid reading", false, "Skipped — init failed");
        check("Current readings stable", false, "Skipped — init failed");
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    float current = currentSensor.readCurrentMa();
    bool readable = (current >= 0.0f && current < 50000.0f); // mA sanity range

    char detail[64];
    snprintf(detail, sizeof(detail), "Read %.1f mA", current);
    check("INA226 returns valid reading", readable, detail);

    // Take 10 readings, check stability
    float minC = 999999.0f, maxC = -999999.0f;
    for (int i = 0; i < 10; i++) {
        float c = currentSensor.readCurrentMa();
        if (c < minC) minC = c;
        if (c > maxC) maxC = c;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    float spread = maxC - minC;
    snprintf(detail, sizeof(detail), "Spread=%.1f mA (min=%.1f max=%.1f)", spread, minC, maxC);
    check("Current readings stable (spread < 500 mA)", spread < 500.0f, detail);
}

// ============================================================================
// Test 2: Limit switch reading
// ============================================================================
static void testLimitSwitch() {
    Serial.println("\n--- Test: Limit Switch ---");

    pinMode(cfg::pin::LIMIT_SWITCH, INPUT_PULLUP);
    vTaskDelay(pdMS_TO_TICKS(50));

    bool state = digitalRead(cfg::pin::LIMIT_SWITCH);
    char detail[64];

    // NC switch: HIGH = not triggered, LOW = triggered
    snprintf(detail, sizeof(detail), "Pin %d = %s",
             cfg::pin::LIMIT_SWITCH, state ? "HIGH (not pressed)" : "LOW (pressed)");
    check("Limit switch readable", true, detail);

    // Read 100 times to check for noise
    uint8_t highCount = 0;
    for (int i = 0; i < 100; i++) {
        if (digitalRead(cfg::pin::LIMIT_SWITCH)) highCount++;
        delayMicroseconds(100);
    }
    snprintf(detail, sizeof(detail), "%d/100 reads HIGH", highCount);
    bool stable = (highCount > 95 || highCount < 5);
    check("Limit switch stable (no bounce at rest)", stable, detail);
}

// ============================================================================
// Test 3: Fault latching and reset
// ============================================================================
static void testFaultLatching() {
    Serial.println("\n--- Test: Fault Latching ---");

    // Ensure no faults initially
    // Need to check via shared state since FaultManager doesn't expose hasFault()
    FaultCode faults = sharedState.getFaults();
    check("No faults initially", faults == FaultCode::NONE);

    // Trigger an overcurrent fault
    faultMgr.triggerFault(FaultCode::OVERCURRENT);

    bool faultLatched = faultMgr.isFaultLatched();
    check("Fault active after trigger", faultLatched);

    faults = sharedState.getFaults();
    bool correctCode = hasFault(faults, FaultCode::OVERCURRENT);
    check("OVERCURRENT fault code latched", correctCode);

    // Verify system state changed to FAULT
    SystemState state = sharedState.getState();
    check("System state is FAULT", state == SystemState::FAULT);

    // Verify motor was emergency-stopped
    // (mode should be STOPPED after emergencyStop)
    check("Motor mode STOPPED after fault", motionCtrl.mode() == MotionController::Mode::STOPPED);

    // Trigger a second fault — both should be latched
    faultMgr.triggerFault(FaultCode::STALL_DETECTED);
    faults = sharedState.getFaults();
    bool hasOC = hasFault(faults, FaultCode::OVERCURRENT);
    bool hasStall = hasFault(faults, FaultCode::STALL_DETECTED);
    check("Multiple faults: OC + STALL both latched", hasOC && hasStall);

    // Reset fault — should succeed since no actual current flowing
    // Re-enable motor first so sensor reads low
    motionCtrl.begin(); // re-init
    vTaskDelay(pdMS_TO_TICKS(100));

    bool resetOk = faultMgr.resetFault();
    check("Fault reset succeeds", resetOk);

    if (resetOk) {
        faults = sharedState.getFaults();
        check("All faults cleared after reset", faults == FaultCode::NONE);
        check("Fault latch cleared", !faultMgr.isFaultLatched());
    } else {
        check("All faults cleared after reset", false, "Reset was refused");
        check("Fault latch cleared", false, "Reset was refused");
    }
}

// ============================================================================
// Test 4: Stall detection — drive motor while blocked
// ============================================================================
static void testStallDetection() {
    Serial.println("\n--- Test: Stall Detection ---");
    Serial.println("  NOTE: Motor shaft should be BLOCKED for this test.");
    Serial.println("  Driving motor toward target for up to 3 seconds.");
    Serial.println("  Stall fault should trigger within ~500 ms.\n");

    // Reset everything
    sharedState.setFaults(FaultCode::NONE);
    sharedState.setState(SystemState::IDLE);
    motionCtrl.begin();
    motionCtrl.resetEncoder();
    vTaskDelay(pdMS_TO_TICKS(100));

    // Set a target far away — PID will drive motor
    motionCtrl.setTarget(cfg::motion::SOFT_LIMIT_MAX);
    sharedState.setState(SystemState::MOVING);

    uint32_t startMs = millis();
    bool stallDetected = false;
    uint32_t stallMs = 0;

    // Run motion + fault evaluation loop
    while ((millis() - startMs) < 3000) {
        motionCtrl.update();
        faultMgr.evaluate();

        // Check if stall fault was raised
        FaultCode faults = sharedState.getFaults();
        if (hasFault(faults, FaultCode::STALL_DETECTED)) {
            stallDetected = true;
            stallMs = millis() - startMs;
            break;
        }

        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    motionCtrl.stop();

    if (stallDetected) {
        char detail[64];
        snprintf(detail, sizeof(detail), "Detected at %lu ms (config=%lu ms)",
                 stallMs, (uint32_t)cfg::safety::STALL_TIME_MS);
        check("Stall fault triggered", true, detail);

        // Verify it triggered within reasonable time
        bool timing = (stallMs >= cfg::safety::STALL_TIME_MS &&
                       stallMs <= cfg::safety::STALL_TIME_MS + 500);
        check("Stall timing within 500 ms of configured value", timing, detail);
    } else {
        check("Stall fault triggered", false,
              "Motor may not be blocked, or encoder is producing edges");
        check("Stall timing within 500 ms of configured value", false, "No stall detected");
    }

    // Clean up
    faultMgr.resetFault();
    motionCtrl.begin();
}

// ============================================================================
// Test 5: Motion timeout — verify timeout fires (takes ~15 sec)
// ============================================================================
static void testMotionTimeout() {
    Serial.println("\n--- Test: Motion Timeout ---");
    Serial.printf("  Driving motor toward unreachable target for %lu+ seconds.\n",
                  (uint32_t)(cfg::motion::TIMEOUT_MS / 1000));
    Serial.println("  Motor shaft should be BLOCKED.\n");

    // Reset
    sharedState.setFaults(FaultCode::NONE);
    sharedState.setState(SystemState::IDLE);
    motionCtrl.begin();
    motionCtrl.resetEncoder();
    vTaskDelay(pdMS_TO_TICKS(100));

    // Set target — PID drives motor
    motionCtrl.setTarget(cfg::motion::SOFT_LIMIT_MAX);
    sharedState.setState(SystemState::MOVING);

    // NOTE: We only call motionCtrl.update(), NOT faultMgr.evaluate(),
    //       so that stall detection doesn't stop the motor before timeout fires.
    uint32_t startMs = millis();
    uint32_t maxWait = cfg::motion::TIMEOUT_MS + 3000;
    bool timedOut = false;

    uint32_t lastProgressMs = 0;
    while ((millis() - startMs) < maxWait) {
        motionCtrl.update();

        // Print progress every 2 seconds
        if ((millis() - lastProgressMs) > 2000) {
            lastProgressMs = millis();
            Serial.printf("  ... %lu / %lu ms  pos=%ld\n",
                          millis() - startMs, (uint32_t)cfg::motion::TIMEOUT_MS,
                          motionCtrl.position());
        }

        if (motionCtrl.timedOut()) {
            timedOut = true;
            break;
        }

        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    motionCtrl.stop();

    uint32_t elapsed = millis() - startMs;
    char detail[80];
    snprintf(detail, sizeof(detail), "Fired at %lu ms (config=%lu ms)",
             elapsed, (uint32_t)cfg::motion::TIMEOUT_MS);
    check("Motion timeout triggered", timedOut, detail);

    if (timedOut) {
        bool closeToConfig = (elapsed >= cfg::motion::TIMEOUT_MS &&
                              elapsed <= cfg::motion::TIMEOUT_MS + 1000);
        check("Timeout within 1s of configured value", closeToConfig, detail);
    }

    // Clean up
    motionCtrl.begin();
}

// ============================================================================
void setup() {
    Serial.begin(115200);
    vTaskDelay(pdMS_TO_TICKS(500));

    Serial.println("\n=============================================");
    Serial.println("  Safety Subsystem Validation Test Suite");
    Serial.println("=============================================");
    Serial.println("  IMPORTANT: For stall & timeout tests,");
    Serial.println("  the motor shaft must be BLOCKED.\n");

    motionCtrl.begin();
    faultMgr.begin();

    testCurrentSensor();
    testLimitSwitch();
    testFaultLatching();
    testStallDetection();
    testMotionTimeout();

    Serial.println("\n=============================================");
    Serial.println("  RESULTS SUMMARY");
    Serial.println("=============================================");
    Serial.printf("  Passed: %d\n", passCount);
    Serial.printf("  Failed: %d\n", failCount);
    Serial.printf("  Total:  %d\n", testNumber);
    if (failCount == 0) {
        Serial.println("\n  >>> ALL TESTS PASSED <<<");
    } else {
        Serial.println("\n  >>> FAILURES DETECTED — review above <<<");
    }
    Serial.println("=============================================\n");
}

void loop() {
    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(1000));
}
