#pragma once
// ============================================================================
// status_json.h — Shared JSON serialization helpers for system status
// Used by REST API, MQTT, and any future telemetry consumers
// ============================================================================

#include <ArduinoJson.h>
#include "system_types.h"

// Add fault name strings into a JsonArray
inline void addFaultNames(FaultCode faults, JsonArray& arr) {
    if (hasFault(faults, FaultCode::STALL_DETECTED))    arr.add("stall");
    if (hasFault(faults, FaultCode::MOTION_TIMEOUT))    arr.add("motion_timeout");
    if (hasFault(faults, FaultCode::LIMIT_SWITCH_FAIL)) arr.add("limit_switch_fail");
    if (hasFault(faults, FaultCode::HOMING_FAILED))     arr.add("homing_failed");
    if (hasFault(faults, FaultCode::BROWNOUT))          arr.add("brownout");
    if (hasFault(faults, FaultCode::WATCHDOG_RESET))    arr.add("watchdog_reset");
    if (hasFault(faults, FaultCode::CONFIG_CORRUPT))    arr.add("config_corrupt");
}

// Serialize full system status to a JSON buffer. Returns bytes written.
inline size_t buildStatusJson(const SystemStatus& st, char* buf, size_t buflen) {
    JsonDocument doc;
    doc["state"]        = systemStateToStr(st.state);
    doc["homing_state"] = homingStateToStr(st.homingState);
    doc["position"]     = st.position;
    doc["target"]       = st.targetPos;
    doc["velocity"]     = serialized(String(st.velocity, 1));
    doc["homed"]        = st.homed;
    doc["limit_switch"] = st.limitSwitch;
    doc["faults"]       = static_cast<uint16_t>(st.faults);
    doc["wifi"]         = st.wifiConnected;
    doc["uptime_ms"]    = st.uptimeMs;

    JsonArray faultArr = doc["fault_names"].to<JsonArray>();
    addFaultNames(st.faults, faultArr);

    return serializeJson(doc, buf, buflen);
}
