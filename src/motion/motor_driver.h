#pragma once
// ============================================================================
// motor_driver.h — BTS7960 dual-PWM H-bridge abstraction
// ============================================================================

#include <Arduino.h>
#include "config.h"

class MotorDriver {
public:
    MotorDriver() = default;

    void begin() {
        // Configure enable pins
        pinMode(cfg::pin::MOTOR_R_EN, OUTPUT);
        pinMode(cfg::pin::MOTOR_L_EN, OUTPUT);
        disable();

        // Configure PWM channels
        ledcSetup(cfg::motor::PWM_CHANNEL_R, cfg::motor::PWM_FREQ_HZ, cfg::motor::PWM_RESOLUTION);
        ledcSetup(cfg::motor::PWM_CHANNEL_L, cfg::motor::PWM_FREQ_HZ, cfg::motor::PWM_RESOLUTION);
        ledcAttachPin(cfg::pin::MOTOR_RPWM, cfg::motor::PWM_CHANNEL_R);
        ledcAttachPin(cfg::pin::MOTOR_LPWM, cfg::motor::PWM_CHANNEL_L);

        // Start with zero output
        ledcWrite(cfg::motor::PWM_CHANNEL_R, 0);
        ledcWrite(cfg::motor::PWM_CHANNEL_L, 0);
    }

    void enable() {
        digitalWrite(cfg::pin::MOTOR_R_EN, HIGH);
        digitalWrite(cfg::pin::MOTOR_L_EN, HIGH);
        _enabled = true;
    }

    void disable() {
        ledcWrite(cfg::motor::PWM_CHANNEL_R, 0);
        ledcWrite(cfg::motor::PWM_CHANNEL_L, 0);
        digitalWrite(cfg::pin::MOTOR_R_EN, LOW);
        digitalWrite(cfg::pin::MOTOR_L_EN, LOW);
        _enabled = false;
    }

    // Drive with normalized value: -1.0 (reverse) to +1.0 (forward)
    // Applies deadband compensation internally
    void drive(float normalizedOutput) {
        if (!_enabled) return;

        // Clamp input
        normalizedOutput = constrain(normalizedOutput, -1.0f, 1.0f);

        uint16_t pwm = 0;
        bool forward = (normalizedOutput >= 0.0f);
        float absOutput = fabsf(normalizedOutput);

        if (absOutput > 0.01f) {
            // Map to PWM range with deadband compensation
            pwm = static_cast<uint16_t>(
                cfg::motor::DEADBAND_PWM +
                absOutput * (cfg::motor::PWM_MAX - cfg::motor::DEADBAND_PWM)
            );
            pwm = min(pwm, cfg::motor::PWM_MAX);
        }

        if (pwm == 0) {
            ledcWrite(cfg::motor::PWM_CHANNEL_R, 0);
            ledcWrite(cfg::motor::PWM_CHANNEL_L, 0);
            _driveDir = 0;
        } else if (forward) {
            ledcWrite(cfg::motor::PWM_CHANNEL_L, 0);
            ledcWrite(cfg::motor::PWM_CHANNEL_R, pwm);
            _driveDir = 1;
        } else {
            ledcWrite(cfg::motor::PWM_CHANNEL_R, 0);
            ledcWrite(cfg::motor::PWM_CHANNEL_L, pwm);
            _driveDir = -1;
        }
    }

    // Emergency stop: immediate coast (no braking)
    void emergencyStop() {
        ledcWrite(cfg::motor::PWM_CHANNEL_R, 0);
        ledcWrite(cfg::motor::PWM_CHANNEL_L, 0);
        // Keep enables high for brake, or low for coast
        digitalWrite(cfg::pin::MOTOR_R_EN, LOW);
        digitalWrite(cfg::pin::MOTOR_L_EN, LOW);
        _enabled = false;
        _driveDir = 0;
    }

    bool isEnabled() const { return _enabled; }

    // Current drive direction: +1 forward, -1 reverse, 0 stopped
    int8_t driveDirection() const { return _driveDir; }

private:
    bool   _enabled  = false;
    int8_t _driveDir = 0;
};
