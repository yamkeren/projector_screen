// ============================================================================
// encoder_direction_test.cpp — Verify encoder counts correctly in both directions
//
// Build:   pio run -e encoder_dir_test -t upload
// Monitor: pio device monitor -e encoder_dir_test
//
// Procedure:
//   1. Drives motor forward at 40% for 2 seconds, records position
//   2. Stops for 1 second, checks for drift
//   3. Drives motor reverse at 40% for 2 seconds, records position
//   4. Stops for 1 second, checks for drift
//   5. Reports pass/fail for each phase
// ============================================================================

#include <Arduino.h>
#include <esp_task_wdt.h>
#include "config.h"
#include "motion/encoder.h"
#include "motion/motor_driver.h"

static SingleChannelEncoder encoder;
static MotorDriver motor;

struct PhaseResult {
    const char* name;
    int32_t startPos;
    int32_t endPos;
    int32_t delta;
    bool    passed;
};

static PhaseResult results[4];
static uint8_t resultCount = 0;

static void runPhase(const char* name, float pwm, int8_t expectedDir, uint32_t durationMs) {
    PhaseResult r;
    r.name = name;

    // Set encoder direction based on expected motor direction
    if (pwm > 0.01f)       encoder.setDirection(1);
    else if (pwm < -0.01f) encoder.setDirection(-1);
    else                    encoder.setDirection(0);

    r.startPos = encoder.read();

    uint32_t start = millis();
    motor.drive(pwm);

    while ((millis() - start) < durationMs) {
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    motor.drive(0.0f);
    encoder.setDirection(0);
    vTaskDelay(pdMS_TO_TICKS(100)); // settle

    r.endPos = encoder.read();
    r.delta = r.endPos - r.startPos;

    if (expectedDir > 0) {
        r.passed = (r.delta > 0);
    } else if (expectedDir < 0) {
        r.passed = (r.delta < 0);
    } else {
        // Expecting no movement — allow ±1 count noise
        r.passed = (abs(r.delta) <= 1);
    }

    results[resultCount++] = r;

    Serial.printf("[%s] start=%ld end=%ld delta=%ld %s\n",
                  r.name, r.startPos, r.endPos, r.delta,
                  r.passed ? "PASS" : "** FAIL **");
}

void setup() {
    Serial.begin(115200);
    vTaskDelay(pdMS_TO_TICKS(500));

    Serial.println("\n=========================================");
    Serial.println("  Encoder Direction Validation Test");
    Serial.println("=========================================\n");

    encoder.begin(cfg::pin::ENCODER_PIN);
    motor.begin();
    motor.enable();

    // Phase 1: Forward drive
    Serial.println("Phase 1: Forward 40% for 2s");
    runPhase("FWD_DRIVE", 0.40f, +1, 2000);

    // Phase 2: Stop — check no drift
    Serial.println("Phase 2: Stopped for 1s (drift check)");
    runPhase("FWD_STOP", 0.0f, 0, 1000);

    // Phase 3: Reverse drive
    Serial.println("Phase 3: Reverse 40% for 2s");
    runPhase("REV_DRIVE", -0.40f, -1, 2000);

    // Phase 4: Stop — check no drift
    Serial.println("Phase 4: Stopped for 1s (drift check)");
    runPhase("REV_STOP", 0.0f, 0, 1000);

    // Summary
    Serial.println("\n=========================================");
    Serial.println("  RESULTS SUMMARY");
    Serial.println("=========================================");
    uint8_t passCount = 0;
    for (uint8_t i = 0; i < resultCount; i++) {
        Serial.printf("  %-12s delta=%+5ld  %s\n",
                      results[i].name, results[i].delta,
                      results[i].passed ? "PASS" : "FAIL");
        if (results[i].passed) passCount++;
    }
    Serial.printf("\n  %d/%d passed\n", passCount, resultCount);

    if (passCount == resultCount) {
        Serial.println("  >>> ALL TESTS PASSED <<<");
    } else {
        Serial.println("  >>> FAILURES DETECTED <<<");
        Serial.println("  Check motor wiring, encoder pin, PWM polarity");
    }
    Serial.println("=========================================\n");
}

void loop() {
    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(1000));
}
