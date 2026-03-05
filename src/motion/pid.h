#pragma once
// ============================================================================
// pid.h — PID controller with anti-windup, output saturation,
//          derivative filtering, and fixed timestep
// ============================================================================

#include <cmath>
#include <algorithm>

class PIDController {
public:
    struct Params {
        float kp         = 0.0f;
        float ki         = 0.0f;
        float kd         = 0.0f;
        float outputMin  = -1.0f;
        float outputMax  = 1.0f;
        float integralMax = 0.5f;    // Anti-windup clamp
        float derivAlpha = 0.1f;     // Derivative low-pass coefficient
        float dt         = 0.005f;   // Fixed timestep in seconds
    };

    explicit PIDController(const Params& p) : _p(p) {}

    PIDController() = default;

    void configure(const Params& p) { _p = p; }

    void reset() {
        _integral     = 0.0f;
        _prevError    = 0.0f;
        _derivFiltered = 0.0f;
        _output       = 0.0f;
    }

    // Returns normalized output in [outputMin, outputMax]
    float compute(float setpoint, float measurement) {
        float error = setpoint - measurement;

        // Proportional
        float pTerm = _p.kp * error;

        // Integral with anti-windup clamping
        _integral += error * _p.dt;
        _integral = std::clamp(_integral, -_p.integralMax, _p.integralMax);
        float iTerm = _p.ki * _integral;

        // Derivative with low-pass filter (on error, not measurement, for simplicity)
        float rawDeriv = (error - _prevError) / _p.dt;
        _derivFiltered = _derivFiltered + _p.derivAlpha * (rawDeriv - _derivFiltered);
        float dTerm = _p.kd * _derivFiltered;

        _prevError = error;

        // Output saturation
        _output = std::clamp(pTerm + iTerm + dTerm, _p.outputMin, _p.outputMax);

        return _output;
    }

    float lastOutput() const { return _output; }
    float lastError() const { return _prevError; }
    float integralValue() const { return _integral; }

private:
    Params _p             = {};
    float  _integral      = 0.0f;
    float  _prevError     = 0.0f;
    float  _derivFiltered = 0.0f;
    float  _output        = 0.0f;
};
