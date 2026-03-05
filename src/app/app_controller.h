#pragma once
// ============================================================================
// app_controller.h — Application-level state machine and command dispatcher
// Coordinates all subsystems, processes commands from queue
// ============================================================================

#include <Arduino.h>
#include "config.h"
#include "core/system_types.h"
#include "core/shared_state.h"
#include "motion/motion_controller.h"
#include "safety/fault_manager.h"
#include "storage/persistent_storage.h"
#include "app/homing_machine.h"
#include "audio/buzzer.h"

class AppController {
public:
    AppController(SharedState& shared, MotionController& motion,
                  FaultManager& faults, PersistentStorage& storage,
                  HomingMachine& homing)
        : _shared(shared), _motion(motion), _faults(faults),
          _storage(storage), _homing(homing) {}

    void begin() {
        // Try to restore position from NVS
        PersistentStorage::StoredConfig sc;
        if (_storage.loadConfig(sc)) {
            if (sc.homed) {
                _motion.setEncoderValue(sc.position);
                _shared.setPosition(sc.position);
                _shared.setHomed(true);
                _shared.setState(SystemState::IDLE);
                log_i("Restored position: %d (homed)", sc.position);
            } else {
                _shared.setState(SystemState::BOOT);
                log_i("Config loaded but not homed — homing required");
            }
        } else {
            _shared.addFault(FaultCode::CONFIG_CORRUPT);
            _shared.setState(SystemState::BOOT);
            log_w("NVS config corrupt or empty — defaulting");
        }
    }

    // Called from AppStateTask
    void update() {
        // Process commands from queue
        Command cmd;
        while (_shared.receiveCommand(cmd)) {
            processCommand(cmd);
        }

        // Periodic position persistence
        SystemState state = _shared.getState();
        if (state == SystemState::IDLE || state == SystemState::MOVING) {
            _storage.savePosition(_motion.position(), _shared.isHomed());
        }

        // Update uptime
        _shared.setUptimeMs(millis());
    }

    // Called from MotionTask to update homing (runs at deterministic rate)
    void updateHoming() {
        _homing.update();
    }

    // Force save before sleep
    void prepareForSleep() {
        _storage.forceSave(_motion.position(), _shared.isHomed());
    }

    void setBuzzer(Buzzer* bz) { _buzzer = bz; }

private:
    SharedState&      _shared;
    MotionController& _motion;
    FaultManager&     _faults;
    PersistentStorage& _storage;
    HomingMachine&    _homing;
    Buzzer*           _buzzer = nullptr;

    void processCommand(const Command& cmd) {
        SystemState state = _shared.getState();

        switch (cmd.type) {
            case CommandType::CLOSE:
                if (state == SystemState::FAULT) break;
                if (!_shared.isHomed()) {
                    log_w("Cannot move — not homed");
                    break;
                }
                _motion.setTarget(cfg::motion::SCREEN_CLOSE_POS);
                _shared.setTargetPosition(cfg::motion::SCREEN_CLOSE_POS);
                _shared.setState(SystemState::MOVING);
                if (_buzzer) _buzzer->playScreenClosing();
                break;

            case CommandType::OPEN:
                if (state == SystemState::FAULT) break;
                if (!_shared.isHomed()) {
                    log_w("Cannot move — not homed");
                    break;
                }
                _motion.setTarget(cfg::motion::SCREEN_OPEN_POS);
                _shared.setTargetPosition(cfg::motion::SCREEN_OPEN_POS);
                _shared.setState(SystemState::MOVING);
                if (_buzzer) _buzzer->playScreenDown();
                break;

            case CommandType::STOP:
                _motion.stop();
                if (state != SystemState::FAULT) {
                    _shared.setState(SystemState::IDLE);
                }
                break;

            case CommandType::HOME:
                if (state == SystemState::FAULT) break;
                _homing.reset();
                _homing.start();
                break;

            case CommandType::RESET_FAULT:
                _faults.resetFault();
                _homing.reset();
                break;

            case CommandType::ENTER_SLEEP:
                if (state != SystemState::MOVING && state != SystemState::HOMING) {
                    prepareForSleep();
                    _shared.setEvent(EVT_WAKE_REQUEST); // Signal PowerTask
                }
                break;

            case CommandType::WAKE:
                _shared.setEvent(EVT_WAKE_REQUEST);
                break;

            case CommandType::NONE:
                break;
        }
    }
};
