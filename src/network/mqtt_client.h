#pragma once
// ============================================================================
// mqtt_client.h — MQTT communication layer
//
// Replaces the REST API with pub/sub messaging:
//   Subscribe: projector/cmd        — receives "up", "down", "stop", "home", "reset"
//   Publish:   projector/status     — periodic JSON telemetry
//   Publish:   projector/state      — on-change state string
//   Publish:   projector/fault      — on-change fault detail
//   LWT:       projector/available  — "online" / "offline"
//
// Thread-safety: all public methods are called only from NetworkTask.
// Commands are forwarded to AppController via SharedState command queue.
// ============================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "config.h"
#include "core/system_types.h"
#include "core/shared_state.h"

class MqttClient {
public:
    explicit MqttClient(SharedState& shared)
        : _shared(shared), _mqtt(_wifiClient) {}

    // Call once after WiFi is connected — configures broker and subscribes
    void begin() {
        _mqtt.setServer(cfg::mqtt::BROKER_HOST, cfg::mqtt::BROKER_PORT);
        _mqtt.setKeepAlive(cfg::mqtt::KEEPALIVE_SEC);
        _mqtt.setBufferSize(512);

        // PubSubClient callback must be a std::function or static — use lambda via setCallback
        _mqtt.setCallback([this](char* topic, byte* payload, unsigned int len) {
            onMessage(topic, payload, len);
        });

        _lastReconnectMs = 0;
        _running = true;
    }

    void stop() {
        if (_mqtt.connected()) {
            // Publish offline before disconnect
            _mqtt.publish(cfg::mqtt::TOPIC_LWT, cfg::mqtt::LWT_OFFLINE, true);
            _mqtt.disconnect();
        }
        _running = false;
    }

    // Non-blocking loop — call from NetworkTask every 10ms
    void update() {
        if (!_running) return;

        if (!_mqtt.connected()) {
            reconnect();
        }

        if (_mqtt.connected()) {
            _mqtt.loop();
            publishStatusPeriodic();
            publishStateOnChange();
        }
    }

    bool isConnected() { return _mqtt.connected(); }
    bool isRunning() const { return _running; }

private:
    SharedState&  _shared;
    WiFiClient    _wifiClient;
    PubSubClient  _mqtt;
    bool          _running = false;

    uint32_t _lastReconnectMs  = 0;
    uint32_t _lastPublishMs    = 0;
    SystemState _prevState     = SystemState::BOOT;
    FaultCode   _prevFaults    = FaultCode::NONE;

    // --- Connection management (non-blocking) -------------------------------

    void reconnect() {
        uint32_t now = millis();
        if ((now - _lastReconnectMs) < cfg::mqtt::RECONNECT_MS) return;
        _lastReconnectMs = now;

        log_i("MQTT: connecting to %s:%d...", cfg::mqtt::BROKER_HOST, cfg::mqtt::BROKER_PORT);

        // LWT: broker publishes "offline" on unclean disconnect
        bool ok;
        if (strlen(cfg::mqtt::USERNAME) > 0) {
            ok = _mqtt.connect(
                cfg::mqtt::CLIENT_ID,
                cfg::mqtt::USERNAME,
                cfg::mqtt::PASSWORD,
                cfg::mqtt::TOPIC_LWT,
                0, true,
                cfg::mqtt::LWT_OFFLINE
            );
        } else {
            ok = _mqtt.connect(
                cfg::mqtt::CLIENT_ID,
                cfg::mqtt::TOPIC_LWT,
                0, true,
                cfg::mqtt::LWT_OFFLINE
            );
        }

        if (ok) {
            log_i("MQTT: connected");
            _mqtt.publish(cfg::mqtt::TOPIC_LWT, cfg::mqtt::LWT_ONLINE, true);
            _mqtt.subscribe(cfg::mqtt::TOPIC_CMD, cfg::mqtt::QOS_CMD);

            // Force immediate status publish on connect
            _lastPublishMs = 0;
            _prevState = SystemState::BOOT;
            _prevFaults = FaultCode::NONE;
        } else {
            log_w("MQTT: connect failed, rc=%d", _mqtt.state());
        }
    }

    // --- Inbound message handler --------------------------------------------

    void onMessage(const char* topic, const uint8_t* payload, unsigned int len) {
        // Safely extract command string — avoid heap String
        char cmd[32];
        size_t copyLen = (len < sizeof(cmd) - 1) ? len : sizeof(cmd) - 1;
        memcpy(cmd, payload, copyLen);
        cmd[copyLen] = '\0';

        // Normalize to lowercase in-place
        for (size_t i = 0; i < copyLen; i++) {
            if (cmd[i] >= 'A' && cmd[i] <= 'Z') cmd[i] += 32;
        }

        log_i("MQTT: cmd received: %s", cmd);

        Command command;

        if (strcmp(cmd, "open") == 0) {
            command = {CommandType::OPEN, cfg::motion::SCREEN_OPEN_POS};
        } else if (strcmp(cmd, "close") == 0) {
            command = {CommandType::CLOSE, cfg::motion::SCREEN_CLOSE_POS};
        } else if (strcmp(cmd, "stop") == 0) {
            command = {CommandType::STOP, 0};
        } else if (strcmp(cmd, "home") == 0) {
            command = {CommandType::HOME, 0};
        } else if (strcmp(cmd, "reset") == 0) {
            command = {CommandType::RESET_FAULT, 0};
        } else if (strcmp(cmd, "sleep") == 0) {
            command = {CommandType::ENTER_SLEEP, 0};
        } else {
            log_w("MQTT: unknown command: %s", cmd);
            return;
        }

        if (!_shared.sendCommand(command)) {
            log_w("MQTT: command queue full, dropping: %s", cmd);
        }
    }

    // --- Outbound telemetry -------------------------------------------------

    void publishStatusPeriodic() {
        uint32_t now = millis();
        if ((now - _lastPublishMs) < cfg::mqtt::PUBLISH_INTERVAL_MS) return;
        _lastPublishMs = now;

        SystemStatus st = _shared.getStatus();

        // Build JSON into stack buffer
        char buf[384];
        JsonDocument doc;
        doc["state"]        = systemStateToStr(st.state);
        doc["homing"]       = homingStateToStr(st.homingState);
        doc["pos"]          = st.position;
        doc["target"]       = st.targetPos;
        doc["vel"]          = serialized(String(st.velocity, 1));
        doc["mA"]           = serialized(String(st.currentMa, 1));
        doc["homed"]        = st.homed;
        doc["limit"]        = st.limitSwitch;
        doc["faults"]       = static_cast<uint16_t>(st.faults);
        doc["uptime"]       = st.uptimeMs;

        // Fault name array
        JsonArray fa = doc["fault_names"].to<JsonArray>();
        if (hasFault(st.faults, FaultCode::OVERCURRENT))       fa.add("overcurrent");
        if (hasFault(st.faults, FaultCode::STALL_DETECTED))    fa.add("stall");
        if (hasFault(st.faults, FaultCode::MOTION_TIMEOUT))    fa.add("motion_timeout");
        if (hasFault(st.faults, FaultCode::LIMIT_SWITCH_FAIL)) fa.add("limit_switch_fail");
        if (hasFault(st.faults, FaultCode::HOMING_FAILED))     fa.add("homing_failed");
        if (hasFault(st.faults, FaultCode::SENSOR_FAULT))      fa.add("sensor_fault");
        if (hasFault(st.faults, FaultCode::BROWNOUT))          fa.add("brownout");
        if (hasFault(st.faults, FaultCode::CONFIG_CORRUPT))    fa.add("config_corrupt");

        size_t len = serializeJson(doc, buf, sizeof(buf));
        _mqtt.publish(cfg::mqtt::TOPIC_STATUS, buf, cfg::mqtt::RETAIN_STATUS);
    }

    // Publish state and fault changes immediately (not just on timer)
    void publishStateOnChange() {
        SystemState curState = _shared.getState();
        FaultCode   curFaults = _shared.getFaults();

        if (curState != _prevState) {
            _prevState = curState;
            _mqtt.publish(cfg::mqtt::TOPIC_STATE, systemStateToStr(curState), true);
        }

        if (curFaults != _prevFaults) {
            _prevFaults = curFaults;
            // Publish fault bitmask as decimal string
            char fBuf[8];
            snprintf(fBuf, sizeof(fBuf), "%u", static_cast<uint16_t>(curFaults));
            _mqtt.publish(cfg::mqtt::TOPIC_FAULT, fBuf, true);
        }
    }

    // --- Helpers ------------------------------------------------------------

    static const char* systemStateToStr(SystemState s) {
        switch (s) {
            case SystemState::BOOT:     return "boot";
            case SystemState::HOMING:   return "homing";
            case SystemState::IDLE:     return "idle";
            case SystemState::MOVING:   return "moving";
            case SystemState::FAULT:    return "fault";
            case SystemState::SLEEPING: return "sleeping";
            default:                    return "unknown";
        }
    }

    static const char* homingStateToStr(HomingState s) {
        switch (s) {
            case HomingState::UNINITIALIZED:  return "uninitialized";
            case HomingState::SEEK_SWITCH:    return "seek_switch";
            case HomingState::VERIFY_CONTACT: return "verify_contact";
            case HomingState::BACK_OFF:       return "back_off";
            case HomingState::APPROACH_SLOW:  return "approach_slow";
            case HomingState::SET_ZERO:       return "set_zero";
            case HomingState::COMPLETE:       return "complete";
            case HomingState::FAILED:         return "failed";
            default:                          return "unknown";
        }
    }
};
