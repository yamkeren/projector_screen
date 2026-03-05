#pragma once
// ============================================================================
// homing_machine.h — Industrial homing state machine with full fault handling
// ============================================================================

#include <Arduino.h>
#include "config.h"
#include "core/system_types.h"
#include "core/shared_state.h"
#include "motion/motion_controller.h"
#include "audio/buzzer.h"

class HomingMachine {
public:
    HomingMachine(SharedState& shared, MotionController& motion)
        : _shared(shared), _motion(motion) {}

    void start() {
        _state = HomingState::SEEK_SWITCH;
        _phaseStartMs = millis();
        _shared.setHomingState(_state);
        _shared.setState(SystemState::HOMING);

        // Drive toward limit switch (negative direction = toward home)
        _motion.setOpenLoopSpeed(-cfg::homing::SEEK_SPEED);

        if (_buzzer) _buzzer->playHoming();
    }

    // Called from MotionTask at 5ms rate — deterministic
    void update() {
        if (_state == HomingState::COMPLETE || _state == HomingState::FAILED ||
            _state == HomingState::UNINITIALIZED) {
            return;
        }

        bool switchActive = (digitalRead(cfg::pin::LIMIT_SWITCH) == LOW);
        float velocity = _motion.velocity();
        uint32_t elapsed = millis() - _phaseStartMs;

        switch (_state) {
            case HomingState::SEEK_SWITCH:
                handleSeekSwitch(switchActive, velocity, elapsed);
                break;

            case HomingState::VERIFY_CONTACT:
                handleVerifyContact(switchActive, elapsed);
                break;

            case HomingState::BACK_OFF:
                handleBackOff(switchActive, elapsed);
                break;

            case HomingState::APPROACH_SLOW:
                handleApproachSlow(switchActive, velocity, elapsed);
                break;

            case HomingState::SET_ZERO:
                handleSetZero();
                break;

            default:
                break;
        }
    }

    HomingState state() const { return _state; }
    bool isComplete() const { return _state == HomingState::COMPLETE; }
    bool isFailed() const { return _state == HomingState::FAILED; }

    void setBuzzer(Buzzer* bz) { _buzzer = bz; }

    void reset() {
        _state = HomingState::UNINITIALIZED;
        _shared.setHomingState(_state);
    }

private:
    SharedState&     _shared;
    MotionController& _motion;
    Buzzer*          _buzzer       = nullptr;

    HomingState _state        = HomingState::UNINITIALIZED;
    uint32_t    _phaseStartMs = 0;
    int32_t     _backoffStart = 0;

    void transition(HomingState next) {
        _state = next;
        _phaseStartMs = millis();
        _shared.setHomingState(_state);
    }

    void fail() {
        _motion.stop();
        _state = HomingState::FAILED;
        _shared.setHomingState(_state);
        _shared.addFault(FaultCode::HOMING_FAILED);
        _shared.setState(SystemState::FAULT);
        _shared.setEvent(EVT_FAULT_OCCURRED);
    }

    // --- Phase Handlers -----------------------------------------------------

    void handleSeekSwitch(bool switchActive, float velocity, uint32_t elapsed) {
        // Check for timeout
        if (elapsed > cfg::homing::PHASE_TIMEOUT_MS) {
            fail();
            return;
        }

        // Stall detection during seek (motor driving but no movement)
        if (elapsed > 500 && fabsf(velocity) < cfg::motion::VELOCITY_THRESHOLD) {
            fail(); // Mechanical jam
            return;
        }

        if (switchActive) {
            _motion.stop();
            transition(HomingState::VERIFY_CONTACT);
        }
    }

    void handleVerifyContact(bool switchActive, uint32_t elapsed) {
        // Dwell to confirm stable switch contact
        if (elapsed > cfg::homing::VERIFY_DWELL_MS) {
            if (switchActive) {
                // Confirmed — back off
                _backoffStart = _motion.position();
                _motion.setOpenLoopSpeed(cfg::homing::SEEK_SPEED); // Move away from switch
                transition(HomingState::BACK_OFF);
            } else {
                // Switch bounced — re-seek
                _motion.setOpenLoopSpeed(-cfg::homing::SEEK_SPEED);
                transition(HomingState::SEEK_SWITCH);
            }
        }

        // Timeout
        if (elapsed > cfg::homing::PHASE_TIMEOUT_MS) {
            fail();
        }
    }

    void handleBackOff(bool switchActive, uint32_t elapsed) {
        if (elapsed > cfg::homing::PHASE_TIMEOUT_MS) {
            fail();
            return;
        }

        int32_t traveled = abs(_motion.position() - _backoffStart);
        if (traveled >= cfg::homing::BACKOFF_COUNTS) {
            _motion.stop();

            if (!switchActive) {
                // Clear of switch — now approach slowly
                _motion.setOpenLoopSpeed(-cfg::homing::APPROACH_SPEED);
                transition(HomingState::APPROACH_SLOW);
            } else {
                // Still on switch after backoff — mechanical fault
                fail();
            }
        }
    }

    void handleApproachSlow(bool switchActive, float velocity, uint32_t elapsed) {
        if (elapsed > cfg::homing::PHASE_TIMEOUT_MS) {
            fail();
            return;
        }

        // Stall detection
        if (elapsed > 500 && fabsf(velocity) < cfg::motion::VELOCITY_THRESHOLD) {
            fail();
            return;
        }

        if (switchActive) {
            _motion.stop();
            transition(HomingState::SET_ZERO);
        }
    }

    void handleSetZero() {
        // Set encoder to zero — this is the home position
        _motion.resetEncoder();
        _motion.stop();

        _shared.setHomed(true);
        _shared.setPosition(0);

        _state = HomingState::COMPLETE;
        _shared.setHomingState(_state);
        _shared.setState(SystemState::IDLE);
        _shared.setEvent(EVT_HOMING_DONE);
    }
};
