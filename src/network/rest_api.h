#pragma once
// ============================================================================
// rest_api.h — Lightweight REST API server
// Uses WebServer with safe JSON formatting (ArduinoJson, no String fragmentation)
// ============================================================================

#include <Arduino.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include "config.h"
#include "core/system_types.h"
#include "core/shared_state.h"

class RestApi {
public:
    explicit RestApi(SharedState& shared)
        : _shared(shared), _server(cfg::network::HTTP_PORT) {}

    void begin() {
        _server.on("/status", HTTP_GET, [this]() { handleStatus(); });
        _server.on("/open",   HTTP_POST, [this]() { handleOpen(); });
        _server.on("/close",  HTTP_POST, [this]() { handleClose(); });
        _server.on("/stop",   HTTP_POST, [this]() { handleStop(); });
        _server.on("/home",   HTTP_POST, [this]() { handleHome(); });
        _server.on("/reset",  HTTP_POST, [this]() { handleReset(); });
        _server.onNotFound([this]() { handleNotFound(); });
        _server.begin();
        _running = true;
    }

    void stop() {
        _server.stop();
        _running = false;
    }

    // Call from NetworkTask — non-blocking
    void handleClient() {
        if (_running) {
            _server.handleClient();
        }
    }

    bool isRunning() const { return _running; }

private:
    SharedState& _shared;
    WebServer    _server;
    bool         _running = false;

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

    void handleStatus() {
        SystemStatus st = _shared.getStatus();

        JsonDocument doc;
        doc["state"]        = systemStateToStr(st.state);
        doc["homing_state"] = homingStateToStr(st.homingState);
        doc["position"]     = st.position;
        doc["target"]       = st.targetPos;
        doc["velocity"]     = serialized(String(st.velocity, 1));
        doc["current_ma"]   = serialized(String(st.currentMa, 1));
        doc["homed"]        = st.homed;
        doc["limit_switch"] = st.limitSwitch;
        doc["faults"]       = static_cast<uint16_t>(st.faults);
        doc["wifi"]         = st.wifiConnected;
        doc["uptime_ms"]    = st.uptimeMs;

        // Build fault detail array
        JsonArray faultArr = doc["fault_names"].to<JsonArray>();
        if (hasFault(st.faults, FaultCode::OVERCURRENT))       faultArr.add("overcurrent");
        if (hasFault(st.faults, FaultCode::STALL_DETECTED))    faultArr.add("stall");
        if (hasFault(st.faults, FaultCode::MOTION_TIMEOUT))    faultArr.add("motion_timeout");
        if (hasFault(st.faults, FaultCode::LIMIT_SWITCH_FAIL)) faultArr.add("limit_switch_fail");
        if (hasFault(st.faults, FaultCode::HOMING_FAILED))     faultArr.add("homing_failed");
        if (hasFault(st.faults, FaultCode::SENSOR_FAULT))      faultArr.add("sensor_fault");
        if (hasFault(st.faults, FaultCode::BROWNOUT))          faultArr.add("brownout");
        if (hasFault(st.faults, FaultCode::CONFIG_CORRUPT))    faultArr.add("config_corrupt");

        // Serialize to stack buffer — no heap String fragmentation
        char buf[512];
        size_t len = serializeJson(doc, buf, sizeof(buf));
        _server.send(200, "application/json", buf);
    }

    void handleOpen() {
        Command cmd{CommandType::OPEN, cfg::motion::SCREEN_OPEN_POS};
        if (_shared.sendCommand(cmd)) {
            sendOk("opening");
        } else {
            sendError(503, "queue_full");
        }
    }

    void handleClose() {
        Command cmd{CommandType::CLOSE, cfg::motion::SCREEN_CLOSE_POS};
        if (_shared.sendCommand(cmd)) {
            sendOk("closing");
        } else {
            sendError(503, "queue_full");
        }
    }

    void handleStop() {
        Command cmd{CommandType::STOP, 0};
        if (_shared.sendCommand(cmd)) {
            sendOk("stopped");
        } else {
            sendError(503, "queue_full");
        }
    }

    void handleHome() {
        Command cmd{CommandType::HOME, 0};
        if (_shared.sendCommand(cmd)) {
            sendOk("homing");
        } else {
            sendError(503, "queue_full");
        }
    }

    void handleReset() {
        Command cmd{CommandType::RESET_FAULT, 0};
        if (_shared.sendCommand(cmd)) {
            sendOk("reset_requested");
        } else {
            sendError(503, "queue_full");
        }
    }

    void handleNotFound() {
        sendError(404, "not_found");
    }

    void sendOk(const char* action) {
        char buf[64];
        snprintf(buf, sizeof(buf), "{\"status\":\"ok\",\"action\":\"%s\"}", action);
        _server.send(200, "application/json", buf);
    }

    void sendError(int code, const char* msg) {
        char buf[64];
        snprintf(buf, sizeof(buf), "{\"status\":\"error\",\"message\":\"%s\"}", msg);
        _server.send(code, "application/json", buf);
    }
};
