#pragma once
// ============================================================================
// motion_controller.h — High-level position motion controller
// Owns encoder + PID + motor driver, provides target-based control
// ============================================================================

#include <Arduino.h>
#include "config.h"
#include "motion/encoder.h"   // SingleChannelEncoder
#include "motion/pid.h"
#include "motion/motor_driver.h"

class MotionController {
public:
    enum class Mode : uint8_t {
        STOPPED,
        POSITION,       // PID position control
        OPEN_LOOP       // Direct speed command (used during homing)
    };

    MotionController() = default;

    void begin() {
        _encoder.begin(cfg::pin::ENCODER_PIN);
        _motor.begin();

        PIDController::Params pp;
        pp.kp         = cfg::pid::KP;
        pp.ki         = cfg::pid::KI;
        pp.kd         = cfg::pid::KD;
        pp.outputMin  = cfg::pid::OUTPUT_MIN;
        pp.outputMax  = cfg::pid::OUTPUT_MAX;
        pp.integralMax = cfg::pid::INTEGRAL_MAX;
        pp.derivAlpha = cfg::pid::DERIV_FILTER_ALPHA;
        pp.dt         = cfg::pid::DT_SEC;
        _pid.configure(pp);

        _motor.enable();
    }

    // Must be called at fixed dt (5 ms) from MotionTask
    void update() {
        // Feed motor drive direction into single-channel encoder
        _encoder.setDirection(_motor.driveDirection());

        int32_t pos = _encoder.read();

        // Velocity estimation (counts per second)
        _velocity = static_cast<float>(pos - _prevPosition) / cfg::pid::DT_SEC;
        _prevPosition = pos;

        switch (_mode) {
            case Mode::STOPPED:
                _motor.drive(0.0f);
                break;

            case Mode::POSITION: {
                // Enforce soft limits on target
                int32_t clampedTarget = constrain(_target,
                    cfg::motion::SOFT_LIMIT_MIN, cfg::motion::SOFT_LIMIT_MAX);

                float output = _pid.compute(
                    static_cast<float>(clampedTarget),
                    static_cast<float>(pos)
                );

                // Check if within tolerance — stop jittering
                if (abs(pos - clampedTarget) <= cfg::motion::POSITION_TOLERANCE) {
                    _motor.drive(0.0f);
                    _atTarget = true;
                } else {
                    _motor.drive(output);
                    _atTarget = false;
                }

                // Motion timeout detection
                if (!_atTarget) {
                    if (_moveStartMs == 0) {
                        _moveStartMs = millis();
                    } else if ((millis() - _moveStartMs) > cfg::motion::TIMEOUT_MS) {
                        _timedOut = true;
                        stop();
                    }
                } else {
                    _moveStartMs = 0;
                }
                break;
            }

            case Mode::OPEN_LOOP:
                _motor.drive(_openLoopSpeed);
                break;
        }
    }

    // --- Public Interface ---------------------------------------------------

    void setTarget(int32_t target) {
        _target = constrain(target, cfg::motion::SOFT_LIMIT_MIN, cfg::motion::SOFT_LIMIT_MAX);
        _atTarget = false;
        _timedOut = false;
        _moveStartMs = 0;
        _pid.reset();
        _mode = Mode::POSITION;
    }

    void setOpenLoopSpeed(float speed) {
        _openLoopSpeed = constrain(speed, -1.0f, 1.0f);
        _mode = Mode::OPEN_LOOP;
    }

    void stop() {
        _mode = Mode::STOPPED;
        _motor.drive(0.0f);
        _pid.reset();
        _openLoopSpeed = 0.0f;
    }

    void emergencyStop() {
        _mode = Mode::STOPPED;
        _motor.emergencyStop();
        _pid.reset();
        _openLoopSpeed = 0.0f;
    }

    void enableMotor() { _motor.enable(); }
    void disableMotor() { _motor.disable(); }

    // Reconfigure PID gains at runtime (for tuning)
    void reconfigurePid(const PIDController::Params& p) {
        _pid.configure(p);
        _pid.reset();
    }

    // --- Getters ------------------------------------------------------------

    int32_t position() const { return _encoder.read(); }
    int32_t target() const { return _target; }
    float   velocity() const { return _velocity; }
    bool    atTarget() const { return _atTarget; }
    bool    timedOut() const { return _timedOut; }
    Mode    mode() const { return _mode; }

    void resetEncoder() { _encoder.reset(); _prevPosition = 0; }
    void setEncoderValue(int32_t val) { _encoder.write(val); _prevPosition = val; }

    float   pidOutput() const { return _pid.lastOutput(); }

private:
    SingleChannelEncoder _encoder;
    PIDController        _pid;
    MotorDriver          _motor;

    Mode    _mode           = Mode::STOPPED;
    int32_t _target         = 0;
    int32_t _prevPosition   = 0;
    float   _velocity       = 0.0f;
    float   _openLoopSpeed  = 0.0f;
    bool    _atTarget       = false;
    bool    _timedOut       = false;
    uint32_t _moveStartMs   = 0;
};
