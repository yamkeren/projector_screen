#pragma once
// ============================================================================
// fault_manager.h — Centralized fault detection, latching, and escalation
// ============================================================================

#include <Arduino.h>
#include "config.h"
#include "core/system_types.h"
#include "core/shared_state.h"
#include "motion/motion_controller.h"

class FaultManager {
public:
    FaultManager(SharedState& shared, MotionController& motion)
        : _shared(shared), _motion(motion) {}

    void begin() {
        pinMode(cfg::pin::LIMIT_SWITCH, INPUT_PULLUP);
    }

    // Called periodically from SafetyTask
    void evaluate() {
        FaultCode newFaults = FaultCode::NONE;

        // --- Stall detection ------------------------------------------------
        // If motor is driving but encoder velocity is near zero
        SystemState state = _shared.getState();
        float velocity = _motion.velocity();
        _shared.setVelocity(velocity);

        if (state == SystemState::MOVING || state == SystemState::HOMING) {
            if (_motion.mode() != MotionController::Mode::STOPPED) {
                if (fabsf(velocity) < cfg::motion::VELOCITY_THRESHOLD &&
                    fabsf(_motion.pidOutput()) > cfg::safety::MIN_PID_OUTPUT) {
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
        _shared.setFaults(FaultCode::NONE);
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

    uint32_t _stallStartMs     = 0;
    bool     _faultLatched     = false;
};
