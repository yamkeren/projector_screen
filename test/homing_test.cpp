// ============================================================================
// homing_test.cpp — Homing FSM validation on real hardware
//
// Build:   pio run -e homing_test -t upload
// Monitor: pio device monitor -e homing_test
//
// Runs the full homing sequence, printing each state transition and timing.
// Reports pass/fail with detailed diagnostics on failure.
//
// Prerequisites:
//   - Limit switch wired on pin 32 (NC to GND + signal, LOW = pressed)
//   - Motor can physically reach the limit switch
//   - Screen not jammed
// ============================================================================

#include <Arduino.h>
#include <esp_task_wdt.h>
#include "config.h"
#include "core/system_types.h"
#include "core/shared_state.h"
#include "motion/motion_controller.h"
#include "app/homing_machine.h"

static SharedState      sharedState;
static MotionController motionCtrl;
static HomingMachine    homing(sharedState, motionCtrl);

static const char* homingStateStr(HomingState s) {
    switch (s) {
        case HomingState::UNINITIALIZED:   return "UNINITIALIZED";
        case HomingState::SEEK_SWITCH:     return "SEEK_SWITCH";
        case HomingState::VERIFY_CONTACT:  return "VERIFY_CONTACT";
        case HomingState::BACK_OFF:        return "BACK_OFF";
        case HomingState::APPROACH_SLOW:   return "APPROACH_SLOW";
        case HomingState::SET_ZERO:        return "SET_ZERO";
        case HomingState::COMPLETE:        return "COMPLETE";
        case HomingState::FAILED:          return "FAILED";
        default:                           return "UNKNOWN";
    }
}

static HomingState lastState = HomingState::UNINITIALIZED;
static uint32_t stateEntryMs = 0;
static uint32_t homingStartMs = 0;

struct StateLog {
    HomingState state;
    uint32_t    entryMs;
    uint32_t    durationMs;
};

static StateLog stateLog[10];
static uint8_t  stateLogCount = 0;

void setup() {
    Serial.begin(115200);
    vTaskDelay(pdMS_TO_TICKS(500));

    Serial.println("\n=========================================");
    Serial.println("  Homing Sequence Validation Test");
    Serial.println("=========================================\n");

    motionCtrl.begin();
    pinMode(cfg::pin::LIMIT_SWITCH, INPUT_PULLUP);

    // Verify limit switch is readable before starting
    bool switchState = digitalRead(cfg::pin::LIMIT_SWITCH);
    Serial.printf("Limit switch initial state: %s (pin %d = %d)\n",
                  switchState ? "HIGH (not pressed)" : "LOW (pressed)",
                  cfg::pin::LIMIT_SWITCH, switchState);

    if (!switchState) {
        Serial.println("WARNING: Limit switch is already activated.");
        Serial.println("  Homing should back off first. This is fine.\n");
    }

    Serial.println("Starting homing in 2 seconds...\n");
    vTaskDelay(pdMS_TO_TICKS(2000));

    homing.start();
    homingStartMs = millis();
    stateEntryMs = homingStartMs;
    lastState = HomingState::SEEK_SWITCH;

    Serial.printf("[%6lu ms] STATE: %s\n", 0UL, homingStateStr(lastState));
}

void loop() {
    uint32_t now = millis();
    esp_task_wdt_reset();

    // Update motion at 5ms rate
    static uint32_t lastMotionMs = 0;
    if ((now - lastMotionMs) >= 5) {
        lastMotionMs = now;
        motionCtrl.update();
    }

    // Update homing FSM at 10ms rate
    static uint32_t lastHomingMs = 0;
    if ((now - lastHomingMs) >= 10) {
        lastHomingMs = now;
        homing.update();
    }

    // Detect state transitions
    HomingState current = homing.state();
    if (current != lastState) {
        uint32_t elapsed = now - homingStartMs;
        uint32_t stateDur = now - stateEntryMs;

        // Log the completed state
        if (stateLogCount < 10) {
            stateLog[stateLogCount].state = lastState;
            stateLog[stateLogCount].entryMs = stateEntryMs - homingStartMs;
            stateLog[stateLogCount].durationMs = stateDur;
            stateLogCount++;
        }

        Serial.printf("[%6lu ms] STATE: %s -> %s  (was in %s for %lu ms)\n",
                      elapsed,
                      homingStateStr(lastState),
                      homingStateStr(current),
                      homingStateStr(lastState),
                      stateDur);

        // Print extra info on transition
        Serial.printf("           pos=%ld vel=%.1f switch=%d\n",
                      motionCtrl.position(),
                      motionCtrl.velocity(),
                      digitalRead(cfg::pin::LIMIT_SWITCH));

        lastState = current;
        stateEntryMs = now;
    }

    // Check for terminal states
    if (current == HomingState::COMPLETE || current == HomingState::FAILED) {
        // Log final state
        if (stateLogCount < 10) {
            stateLog[stateLogCount].state = current;
            stateLog[stateLogCount].entryMs = stateEntryMs - homingStartMs;
            stateLog[stateLogCount].durationMs = now - stateEntryMs;
            stateLogCount++;
        }

        uint32_t totalMs = now - homingStartMs;

        Serial.println("\n=========================================");
        if (current == HomingState::COMPLETE) {
            Serial.println("  HOMING: PASSED");
        } else {
            Serial.println("  HOMING: FAILED");
        }
        Serial.println("=========================================");
        Serial.printf("  Total time: %lu ms\n", totalMs);
        Serial.printf("  Final pos:  %ld counts\n", motionCtrl.position());

        Serial.println("\n  State Trace:");
        for (uint8_t i = 0; i < stateLogCount; i++) {
            Serial.printf("    [%5lu ms] %-18s  %lu ms\n",
                          stateLog[i].entryMs,
                          homingStateStr(stateLog[i].state),
                          stateLog[i].durationMs);
        }

        if (current == HomingState::FAILED) {
            Serial.println("\n  Troubleshooting:");
            Serial.println("  - Check limit switch wiring (NC to GND + signal)");
            Serial.println("  - Check motor runs in correct direction toward switch");
            Serial.println("  - Check encoder counts change when motor runs");
            Serial.println("  - Increase PHASE_TIMEOUT_MS if travel is long");
            Serial.println("  - Check SEEK_SPEED sign matches direction to switch");
        }

        Serial.println("=========================================\n");

        // Halt — done
        motionCtrl.stop();
        while (true) {
            esp_task_wdt_reset();
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    vTaskDelay(pdMS_TO_TICKS(5));
}
