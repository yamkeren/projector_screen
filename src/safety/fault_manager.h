#pragma once
// ============================================================================
// fault_manager.h — Centralized fault detection, latching, and escalation
// ============================================================================

#include <Arduino.h>
#include "config.h"
#include "core/system_types.h"
#include "core/shared_state.h"
#include "safety/current_sensor.h"
#include "motion/motion_controller.h"

class FaultManager {
public:
    FaultManager(SharedState& shared, MotionController& motion, CurrentSensor& sensor)
        : _shared(shared), _motion(motion), _sensor(sensor) {}

    void begin() {
        pinMode(cfg::pin::LIMIT_SWITCH, INPUT_PULLUP);
    }

    // Called periodically from SafetyTask
    void evaluate() {
        FaultCode newFaults = FaultCode::NONE;

        // --- Overcurrent detection with filter counter ----------------------
        float currentMa = _sensor.readCurrentMa();
        _shared.setCurrentMa(currentMa);

        if (currentMa > cfg::safety::OVERCURRENT_MA) {
            _overcurrentCount++;
            if (_overcurrentCount >= cfg::safety::OVERCURRENT_FILTER_COUNT) {
                newFaults |= FaultCode::OVERCURRENT;
            }
        } else {
            _overcurrentCount = 0;
        }

        // --- Stall detection ------------------------------------------------
        // If motor is driving but encoder velocity is near zero
        SystemState state = _shared.getState();
        float velocity = _motion.velocity();
        _shared.setVelocity(velocity);

        if (state == SystemState::MOVING || state == SystemState::HOMING) {
            if (_motion.mode() != MotionController::Mode::STOPPED) {
                if (fabsf(velocity) < cfg::motion::VELOCITY_THRESHOLD &&
                    fabsf(_motion.pidOutput()) > 0.15f) {
                    if (_stallStartMs == 0) {
                        _stallStartMs = millis();
                    } else if ((millis() - _stallStartMs) > cfg::safety::STALL_TIME_MS) {
                        newFaults |= FaultCode::STALL_DETECTED;
                    }
                } else {
                    _stallStartMs = 0;
                }
            } else {
                _stallStartMs = 0;
            }
        }

        // --- Limit switch validation ----------------------------------------
        bool switchActive = (digitalRead(cfg::pin::LIMIT_SWITCH) == LOW); // NC switch
        _shared.setLimitSwitch(switchActive);

        // --- Motion timeout -------------------------------------------------
        if (_motion.timedOut()) {
            newFaults |= FaultCode::MOTION_TIMEOUT;
        }

        // --- Sensor fault ---------------------------------------------------
        if (_sensor.hasFault()) {
            newFaults |= FaultCode::SENSOR_FAULT;
        }

        // --- Latch any new faults -------------------------------------------
        if (newFaults != FaultCode::NONE) {
            triggerFault(newFaults);
        }

        // Update position in shared state
        _shared.setPosition(_motion.position());
    }

    void triggerFault(FaultCode fault) {
        _shared.addFault(fault);
        _shared.setState(SystemState::FAULT);
        _shared.setEvent(EVT_FAULT_OCCURRED);

        // Immediate motor shutdown
        _motion.emergencyStop();

        _faultLatched = true;
    }

    // Explicit reset required — returns true if reset was successful
    bool resetFault() {
        // Only reset if underlying condition is cleared
        float currentMa = _sensor.lastFilteredMa();
        if (currentMa > cfg::safety::WARNING_CURRENT_MA) {
            return false; // Still overcurrent, refuse reset
        }

        _shared.setFaults(FaultCode::NONE);
        _overcurrentCount = 0;
        _stallStartMs = 0;
        _faultLatched = false;
        _shared.clearEvent(EVT_FAULT_OCCURRED);

        // Return to IDLE — homing may be required depending on state
        if (_shared.isHomed()) {
            _shared.setState(SystemState::IDLE);
        } else {
            _shared.setState(SystemState::BOOT);
        }
        return true;
    }

    bool isFaultLatched() const { return _faultLatched; }

private:
    SharedState&     _shared;
    MotionController& _motion;
    CurrentSensor&   _sensor;

    uint8_t  _overcurrentCount = 0;
    uint32_t _stallStartMs     = 0;
    bool     _faultLatched     = false;
};
