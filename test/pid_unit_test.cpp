// ============================================================================
// pid_unit_test.cpp — PID controller unit tests
// Runs on ESP32, outputs results via Serial (115200 baud).
// Build: pio run -e pid_unit_test
// Upload: pio run -e pid_unit_test -t upload
// ============================================================================

#include <Arduino.h>
#include <cmath>
#include "motion/pid.h"
#include "config.h"

// --- Test framework (minimal, self-contained) --------------------------------
static int _passed = 0;
static int _failed = 0;

#define ASSERT_NEAR(actual, expected, tol, msg)                             \
    do {                                                                    \
        float _a = (actual), _e = (expected), _t = (tol);                  \
        if (fabsf(_a - _e) <= _t) {                                        \
            _passed++;                                                      \
            Serial.printf("  PASS: %s  (%.4f ≈ %.4f)\n", msg, _a, _e);    \
        } else {                                                            \
            _failed++;                                                      \
            Serial.printf("  FAIL: %s  (got %.4f, expected %.4f ± %.4f)\n",\
                          msg, _a, _e, _t);                                 \
        }                                                                   \
    } while (0)

#define ASSERT_TRUE(cond, msg)                                              \
    do {                                                                    \
        if (cond) { _passed++; Serial.printf("  PASS: %s\n", msg); }       \
        else      { _failed++; Serial.printf("  FAIL: %s\n", msg); }       \
    } while (0)

// --- Test: Proportional-only step response ----------------------------------
void test_proportional_only() {
    Serial.println("\n[TEST] Proportional only (Ki=0, Kd=0)");
    PIDController pid;
    PIDController::Params p;
    p.kp = 1.0f; p.ki = 0.0f; p.kd = 0.0f;
    p.outputMin = -1.0f; p.outputMax = 1.0f;
    p.integralMax = 1.0f; p.derivAlpha = 0.1f; p.dt = 0.005f;
    pid.configure(p);

    float out = pid.compute(100.0f, 0.0f);
    ASSERT_NEAR(out, 1.0f, 0.001f, "P-only large error saturates to +1");

    pid.reset();
    out = pid.compute(0.0f, 100.0f);
    ASSERT_NEAR(out, -1.0f, 0.001f, "P-only negative error saturates to -1");

    pid.reset();
    out = pid.compute(10.0f, 9.5f);
    ASSERT_NEAR(out, 0.5f, 0.001f, "P-only error=0.5 → output=0.5");

    pid.reset();
    out = pid.compute(10.0f, 10.0f);
    ASSERT_NEAR(out, 0.0f, 0.001f, "P-only zero error → output=0");
}

// --- Test: Integral accumulates and eliminates steady-state error -----------
void test_integral_accumulation() {
    Serial.println("\n[TEST] Integral accumulation");
    PIDController pid;
    PIDController::Params p;
    p.kp = 0.0f; p.ki = 1.0f; p.kd = 0.0f;
    p.outputMin = -10.0f; p.outputMax = 10.0f;
    p.integralMax = 10.0f; p.derivAlpha = 0.1f; p.dt = 0.01f;
    pid.configure(p);

    // Feed constant error=1.0 for 100 steps: integral = 100 * 1.0 * 0.01 = 1.0
    float out = 0;
    for (int i = 0; i < 100; i++) {
        out = pid.compute(1.0f, 0.0f);
    }
    // Ki * integral = 1.0 * 1.0 = 1.0
    ASSERT_NEAR(out, 1.0f, 0.01f, "Integral after 100 steps of error=1.0");
    ASSERT_NEAR(pid.integralValue(), 1.0f, 0.01f, "Raw integral value = 1.0");
}

// --- Test: Anti-windup clamp ------------------------------------------------
void test_anti_windup() {
    Serial.println("\n[TEST] Anti-windup clamping");
    PIDController pid;
    PIDController::Params p;
    p.kp = 0.0f; p.ki = 10.0f; p.kd = 0.0f;
    p.outputMin = -5.0f; p.outputMax = 5.0f;
    p.integralMax = 0.3f; p.derivAlpha = 0.1f; p.dt = 0.01f;
    pid.configure(p);

    // Feed large error for many steps — integral should clamp at ±0.3
    for (int i = 0; i < 1000; i++) {
        pid.compute(100.0f, 0.0f);
    }
    float intVal = pid.integralValue();
    ASSERT_TRUE(intVal <= 0.3f + 0.001f, "Integral clamped at +integralMax");
    ASSERT_TRUE(intVal >= -0.3f - 0.001f, "Integral above -integralMax");

    pid.reset();
    for (int i = 0; i < 1000; i++) {
        pid.compute(-100.0f, 0.0f);
    }
    intVal = pid.integralValue();
    ASSERT_TRUE(intVal >= -0.3f - 0.001f, "Negative integral clamped at -integralMax");
}

// --- Test: Output saturation ------------------------------------------------
void test_output_saturation() {
    Serial.println("\n[TEST] Output saturation");
    PIDController pid;
    PIDController::Params p;
    p.kp = 100.0f; p.ki = 0.0f; p.kd = 0.0f;
    p.outputMin = -0.8f; p.outputMax = 0.8f;
    p.integralMax = 1.0f; p.derivAlpha = 0.1f; p.dt = 0.005f;
    pid.configure(p);

    float out = pid.compute(100.0f, 0.0f);
    ASSERT_NEAR(out, 0.8f, 0.001f, "Positive saturation at outputMax");

    pid.reset();
    out = pid.compute(-100.0f, 0.0f);
    ASSERT_NEAR(out, -0.8f, 0.001f, "Negative saturation at outputMin");
}

// --- Test: Derivative kick filtering ----------------------------------------
void test_derivative_filtering() {
    Serial.println("\n[TEST] Derivative filtering");
    PIDController pid;
    PIDController::Params p;
    p.kp = 0.0f; p.ki = 0.0f; p.kd = 1.0f;
    p.outputMin = -100.0f; p.outputMax = 100.0f;
    p.integralMax = 1.0f;
    p.derivAlpha = 0.1f;  // Heavy filtering
    p.dt = 0.01f;
    pid.configure(p);

    // Step change: error goes from 0 to 10
    // Raw deriv = 10/0.01 = 1000, but filtered should be much smaller on first call
    float out = pid.compute(10.0f, 0.0f);  // First call, prevError was 0
    float rawDeriv = 10.0f / 0.01f; // 1000
    float expectedFiltered = 0.0f + 0.1f * (rawDeriv - 0.0f); // 100
    ASSERT_NEAR(out, 1.0f * expectedFiltered, 0.5f, "Filtered derivative on step");

    // Second call with same error — derivative should decay
    float out2 = pid.compute(10.0f, 0.0f);
    ASSERT_TRUE(fabsf(out2) < fabsf(out), "Derivative decays with constant error");
}

// --- Test: Reset clears state -----------------------------------------------
void test_reset() {
    Serial.println("\n[TEST] Reset clears state");
    PIDController pid;
    PIDController::Params p;
    p.kp = 1.0f; p.ki = 1.0f; p.kd = 1.0f;
    p.outputMin = -10.0f; p.outputMax = 10.0f;
    p.integralMax = 10.0f; p.derivAlpha = 0.5f; p.dt = 0.01f;
    pid.configure(p);

    for (int i = 0; i < 50; i++) pid.compute(5.0f, 0.0f);

    ASSERT_TRUE(pid.integralValue() != 0.0f, "Pre-reset integral nonzero");

    pid.reset();
    ASSERT_NEAR(pid.integralValue(), 0.0f, 0.001f, "Post-reset integral = 0");
    ASSERT_NEAR(pid.lastOutput(), 0.0f, 0.001f, "Post-reset output = 0");
    ASSERT_NEAR(pid.lastError(), 0.0f, 0.001f, "Post-reset error = 0");
}

// --- Test: Simulated step response convergence (P+I) -----------------------
void test_step_convergence() {
    Serial.println("\n[TEST] Step response convergence (P+I)");
    PIDController pid;
    PIDController::Params p;
    p.kp = 0.5f; p.ki = 0.2f; p.kd = 0.01f;
    p.outputMin = -1.0f; p.outputMax = 1.0f;
    p.integralMax = 0.8f; p.derivAlpha = 0.1f; p.dt = 0.005f;
    pid.configure(p);

    // Simple first-order plant simulation: pos += output * gain * dt
    float pos = 0.0f;
    float target = 100.0f;
    float plantGain = 50.0f;  // counts/sec per unit output
    bool converged = false;

    for (int i = 0; i < 10000; i++) {
        float out = pid.compute(target, pos);
        pos += out * plantGain * p.dt;

        if (fabsf(pos - target) < 1.0f) {
            converged = true;
            Serial.printf("  INFO: Converged at step %d (%.1f ms), pos=%.2f\n",
                          i, i * p.dt * 1000.0f, pos);
            break;
        }
    }

    ASSERT_TRUE(converged, "P+I converges to target within 10000 steps");
    ASSERT_NEAR(pos, target, 2.0f, "Final position near target");
}

// --- Test: Simulated step response with different gains --------------------
void test_gain_comparison() {
    Serial.println("\n[TEST] Gain comparison — higher Kp = faster response");

    auto simulate = [](float kp, float ki) -> int {
        PIDController pid;
        PIDController::Params p;
        p.kp = kp; p.ki = ki; p.kd = 0.01f;
        p.outputMin = -1.0f; p.outputMax = 1.0f;
        p.integralMax = 0.8f; p.derivAlpha = 0.1f; p.dt = 0.005f;
        pid.configure(p);

        float pos = 0.0f;
        float target = 50.0f;
        float plantGain = 40.0f;

        for (int i = 0; i < 10000; i++) {
            float out = pid.compute(target, pos);
            pos += out * plantGain * p.dt;
            if (fabsf(pos - target) < 1.0f) return i;
        }
        return 10000;
    };

    int steps_low  = simulate(0.3f, 0.1f);
    int steps_high = simulate(1.5f, 0.1f);

    Serial.printf("  INFO: Low Kp=0.3 → %d steps, High Kp=1.5 → %d steps\n",
                  steps_low, steps_high);
    ASSERT_TRUE(steps_high < steps_low, "Higher Kp converges faster");
}

// --- Test: Default config values step response ------------------------------
void test_default_config_response() {
    Serial.println("\n[TEST] Default config PID step response");
    PIDController pid;
    PIDController::Params p;
    p.kp = cfg::pid::KP;
    p.ki = cfg::pid::KI;
    p.kd = cfg::pid::KD;
    p.outputMin = cfg::pid::OUTPUT_MIN;
    p.outputMax = cfg::pid::OUTPUT_MAX;
    p.integralMax = cfg::pid::INTEGRAL_MAX;
    p.derivAlpha = cfg::pid::DERIV_FILTER_ALPHA;
    p.dt = cfg::pid::DT_SEC;
    pid.configure(p);

    // Simulate with a simple plant — position counts
    float pos = 0.0f;
    float target = static_cast<float>(cfg::motion::SCREEN_OPEN_POS);
    float plantGain = 30.0f;  // rough model
    float maxOvershoot = 0.0f;
    bool converged = false;
    int settleStep = -1;
    int stepsInTol = 0;

    for (int i = 0; i < 20000; i++) {
        float out = pid.compute(target, pos);
        pos += out * plantGain * p.dt;

        float overshoot = pos - target;
        if (overshoot > maxOvershoot) maxOvershoot = overshoot;

        if (fabsf(pos - target) <= static_cast<float>(cfg::motion::POSITION_TOLERANCE)) {
            stepsInTol++;
            if (stepsInTol >= 20 && settleStep < 0) { // Sustained for 20 steps
                settleStep = i;
                converged = true;
            }
        } else {
            stepsInTol = 0;
        }
    }

    Serial.printf("  INFO: Settle step=%d (%.0f ms), overshoot=%.2f counts\n",
                  settleStep, settleStep * p.dt * 1000.0f, maxOvershoot);

    ASSERT_TRUE(converged, "Default config converges");
    // Overshoot should be reasonable (< 20% of travel)
    float pctOvershoot = maxOvershoot / target * 100.0f;
    Serial.printf("  INFO: Overshoot = %.1f%%\n", pctOvershoot);
    ASSERT_TRUE(pctOvershoot < 25.0f, "Overshoot < 25% of target");
}

// ============================================================================
void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println("\n========================================");
    Serial.println("  PID Controller Unit Tests");
    Serial.println("========================================");

    test_proportional_only();
    test_integral_accumulation();
    test_anti_windup();
    test_output_saturation();
    test_derivative_filtering();
    test_reset();
    test_step_convergence();
    test_gain_comparison();
    test_default_config_response();

    Serial.println("\n========================================");
    Serial.printf("  Results: %d PASSED, %d FAILED\n", _passed, _failed);
    Serial.println("========================================\n");

    if (_failed == 0) {
        Serial.println("ALL TESTS PASSED");
    } else {
        Serial.println("SOME TESTS FAILED — review output above");
    }
}

void loop() {
    delay(10000);
}
