// ============================================================================
// pid_tune_live.cpp — Interactive PID tuning tool for ESP32
//
// Runs PID position control on real hardware. Serial commands let you
// adjust gains and trigger step responses in real-time.
//
// Build:   pio run -e pid_tune_live
// Upload:  pio run -e pid_tune_live -t upload
// Monitor: pio device monitor -e pid_tune_live
//
// Serial Commands (115200 baud):
//   kp <val>    — set proportional gain
//   ki <val>    — set integral gain
//   kd <val>    — set derivative gain
//   alpha <val> — set derivative filter coefficient
//   imax <val>  — set integral anti-windup limit
//   step <pos>  — execute step to target position and log response
//   home        — run homing sequence
//   stop        — emergency stop
//   status      — print current gains and position
//   log on/off  — enable/disable continuous position logging (~20 ms)
//   reset       — reset PID and encoder to 0
// ============================================================================

#include <Arduino.h>
#include <esp_task_wdt.h>
#include <cstring>

#include "config.h"
#include "core/system_types.h"
#include "core/shared_state.h"
#include "motion/motion_controller.h"

// --- Subsystems (minimal set for tuning) ------------------------------------
static SharedState       sharedState;
static MotionController  motionCtrl;

// --- Tuning state -----------------------------------------------------------
static PIDController::Params tunePid;
static bool  logEnabled   = false;
static bool  profiling    = false;
static float profileTarget = 0.0f;
static uint32_t profileStartMs = 0;
static float profileMaxOvershoot = 0.0f;
static float profileMinUndershoot = 0.0f;
static bool  profileSettled = false;
static int   profileSettleCount = 0;
static uint32_t profileSettleMs = 0;

// --- Serial command buffer --------------------------------------------------
static char   cmdBuf[64] = {};
static uint8_t cmdIdx = 0;

// --- Timing -----------------------------------------------------------------
static uint32_t lastMotionMs = 0;
static uint32_t lastLogMs    = 0;
static uint32_t lastStatusMs = 0;

// Forward declarations
void processCommand();
void printStatus();
void printGains();
void updateProfile();

// ============================================================================
void setup() {
    Serial.begin(115200);
    delay(300);

    Serial.println("\n=== PID Live Tuning Tool ===");
    Serial.println("Commands: kp/ki/kd/alpha/imax <val>, step <pos>,");
    Serial.println("          home, stop, status, log on/off, reset\n");

    // Initialize with default config gains
    tunePid.kp         = cfg::pid::KP;
    tunePid.ki         = cfg::pid::KI;
    tunePid.kd         = cfg::pid::KD;
    tunePid.outputMin  = cfg::pid::OUTPUT_MIN;
    tunePid.outputMax  = cfg::pid::OUTPUT_MAX;
    tunePid.integralMax = cfg::pid::INTEGRAL_MAX;
    tunePid.derivAlpha = cfg::pid::DERIV_FILTER_ALPHA;
    tunePid.dt         = cfg::pid::DT_SEC;

    motionCtrl.begin();
    // The motion controller already configures PID from config defaults

    printGains();
    Serial.println("System ready. Send 'home' first, then 'step <pos>'.");
}

// ============================================================================
void loop() {
    uint32_t now = millis();

    // --- Motion update (5 ms) -----------------------------------------------
    if ((now - lastMotionMs) >= 5) {
        lastMotionMs = now;
        motionCtrl.update();

        // Update shared state for logging
        sharedState.setPosition(motionCtrl.position());
        sharedState.setVelocity(motionCtrl.velocity());

        // Profile tracking
        if (profiling) {
            updateProfile();
        }

        // Check if motion completed
        if (motionCtrl.atTarget() &&
            sharedState.getState() == SystemState::MOVING) {
            sharedState.setState(SystemState::IDLE);
        }
    }

    // --- Continuous position log (20 ms) ------------------------------------
    if (logEnabled && (now - lastLogMs) >= 20) {
        lastLogMs = now;
        // CSV format: time_ms, position, target, velocity, pidOutput
        Serial.printf("%lu,%ld,%ld,%.1f,%.3f\n",
                      now,
                      motionCtrl.position(),
                      motionCtrl.target(),
                      motionCtrl.velocity(),
                      motionCtrl.pidOutput());
    }

    // --- Serial command parse -----------------------------------------------
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (cmdIdx > 0) {
                cmdBuf[cmdIdx] = '\0';
                processCommand();
                cmdIdx = 0;
            }
        } else if (cmdIdx < sizeof(cmdBuf) - 1) {
            cmdBuf[cmdIdx++] = c;
        }
    }

    // Feed watchdog
    esp_task_wdt_reset();
}

// ============================================================================
// Profile a step response: track overshoot, undershoot, settling time
// ============================================================================
void updateProfile() {
    float pos = static_cast<float>(motionCtrl.position());
    float error = pos - profileTarget;

    if (error > profileMaxOvershoot) profileMaxOvershoot = error;
    if (error < profileMinUndershoot) profileMinUndershoot = error;

    float tol = static_cast<float>(cfg::motion::POSITION_TOLERANCE);
    if (fabsf(error) <= tol) {
        profileSettleCount++;
        if (profileSettleCount >= 40 && !profileSettled) { // 40 * 5ms = 200ms sustained
            profileSettled = true;
            profileSettleMs = millis() - profileStartMs;
        }
    } else {
        profileSettleCount = 0;
    }
}

void endProfile() {
    profiling = false;
    uint32_t elapsed = millis() - profileStartMs;

    Serial.println("\n--- Step Response Profile ---");
    Serial.printf("  Target:       %.0f\n", profileTarget);
    Serial.printf("  Final pos:    %ld\n", motionCtrl.position());
    Serial.printf("  Overshoot:    %.1f counts (%.1f%%)\n",
                  profileMaxOvershoot,
                  profileTarget != 0 ? profileMaxOvershoot / fabsf(profileTarget) * 100.0f : 0);
    Serial.printf("  Undershoot:   %.1f counts\n", profileMinUndershoot);
    if (profileSettled) {
        Serial.printf("  Settle time:  %lu ms\n", profileSettleMs);
    } else {
        Serial.printf("  Settle time:  NOT SETTLED (after %lu ms)\n", elapsed);
    }
    Serial.printf("  Total time:   %lu ms\n", elapsed);
    Serial.println("----------------------------\n");
}

// ============================================================================
void processCommand() {
    // Lowercase the buffer
    for (int i = 0; i < cmdIdx; i++) {
        if (cmdBuf[i] >= 'A' && cmdBuf[i] <= 'Z') cmdBuf[i] += 32;
    }

    float val = 0;

    if (sscanf(cmdBuf, "kp %f", &val) == 1) {
        tunePid.kp = val;
        motionCtrl.reconfigurePid(tunePid);
        Serial.printf("OK: Kp = %.4f (APPLIED)\n", val);
        printGains();

    } else if (sscanf(cmdBuf, "ki %f", &val) == 1) {
        tunePid.ki = val;
        motionCtrl.reconfigurePid(tunePid);
        Serial.printf("OK: Ki = %.4f (APPLIED)\n", val);
        printGains();

    } else if (sscanf(cmdBuf, "kd %f", &val) == 1) {
        tunePid.kd = val;
        motionCtrl.reconfigurePid(tunePid);
        Serial.printf("OK: Kd = %.4f (APPLIED)\n", val);
        printGains();

    } else if (sscanf(cmdBuf, "alpha %f", &val) == 1) {
        tunePid.derivAlpha = val;
        motionCtrl.reconfigurePid(tunePid);
        Serial.printf("OK: derivAlpha = %.4f (APPLIED)\n", val);
        printGains();

    } else if (sscanf(cmdBuf, "imax %f", &val) == 1) {
        tunePid.integralMax = val;
        motionCtrl.reconfigurePid(tunePid);
        Serial.printf("OK: integralMax = %.4f (APPLIED)\n", val);
        printGains();

    } else if (sscanf(cmdBuf, "step %f", &val) == 1) {
        if (profiling) endProfile();
        int32_t target = static_cast<int32_t>(val);
        Serial.printf("STEP → %ld (from %ld)\n", target, motionCtrl.position());

        // Start profiling
        profiling = true;
        profileTarget = val;
        profileStartMs = millis();
        profileMaxOvershoot = 0;
        profileMinUndershoot = 0;
        profileSettled = false;
        profileSettleCount = 0;
        profileSettleMs = 0;

        logEnabled = true;  // Auto-enable logging during step
        Serial.println("t_ms,pos,target,vel,pidOut");  // CSV header

        motionCtrl.setTarget(target);
        sharedState.setTargetPosition(target);
        sharedState.setState(SystemState::MOVING);

    } else if (strncmp(cmdBuf, "home", 4) == 0) {
        Serial.println("Homing not implemented in tuning tool — reset encoder to 0");
        motionCtrl.resetEncoder();
        sharedState.setPosition(0);
        sharedState.setHomed(true);
        sharedState.setState(SystemState::IDLE);
        Serial.println("Position zeroed. Ready for 'step <pos>'");

    } else if (strncmp(cmdBuf, "stop", 4) == 0) {
        motionCtrl.stop();
        if (profiling) endProfile();
        logEnabled = false;
        sharedState.setState(SystemState::IDLE);
        Serial.println("STOPPED");

    } else if (strncmp(cmdBuf, "status", 6) == 0) {
        printStatus();

    } else if (strncmp(cmdBuf, "log on", 6) == 0) {
        logEnabled = true;
        Serial.println("t_ms,pos,target,vel,pidOut");
        Serial.println("Logging ON (CSV)");

    } else if (strncmp(cmdBuf, "log off", 7) == 0) {
        logEnabled = false;
        if (profiling) endProfile();
        Serial.println("Logging OFF");

    } else if (strncmp(cmdBuf, "reset", 5) == 0) {
        motionCtrl.stop();
        motionCtrl.resetEncoder();
        sharedState.setPosition(0);
        if (profiling) endProfile();
        logEnabled = false;
        Serial.println("PID reset, encoder zeroed");

    } else {
        Serial.printf("Unknown: '%s'\n", cmdBuf);
        Serial.println("Commands: kp/ki/kd/alpha/imax <val>, step <pos>");
        Serial.println("          home, stop, status, log on/off, reset");
    }
}

void printGains() {
    Serial.println("--- Current PID Gains ---");
    Serial.printf("  Kp    = %.4f\n", tunePid.kp);
    Serial.printf("  Ki    = %.4f\n", tunePid.ki);
    Serial.printf("  Kd    = %.4f\n", tunePid.kd);
    Serial.printf("  alpha = %.4f\n", tunePid.derivAlpha);
    Serial.printf("  iMax  = %.4f\n", tunePid.integralMax);
    Serial.println("-------------------------");
}

void printStatus() {
    Serial.println("\n--- Status ---");
    Serial.printf("  Position:  %ld\n", motionCtrl.position());
    Serial.printf("  Target:    %ld\n", motionCtrl.target());
    Serial.printf("  Velocity:  %.1f\n", motionCtrl.velocity());
    Serial.printf("  PID out:   %.3f\n", motionCtrl.pidOutput());
    Serial.printf("  At target: %s\n", motionCtrl.atTarget() ? "yes" : "no");
    Serial.printf("  Timed out: %s\n", motionCtrl.timedOut() ? "yes" : "no");
    printGains();
}
