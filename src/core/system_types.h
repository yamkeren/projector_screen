#pragma once
// ============================================================================
// system_types.h — Shared enumerations, structs, and type aliases
// ============================================================================

#include <cstdint>

// --- System States ----------------------------------------------------------
enum class SystemState : uint8_t {
    BOOT,
    HOMING,
    IDLE,
    MOVING,
    FAULT,
    SLEEPING
};

// --- Fault Codes (bit flags for compound faults) ----------------------------
enum class FaultCode : uint16_t {
    NONE              = 0x0000,
    STALL_DETECTED    = 0x0002,
    MOTION_TIMEOUT    = 0x0004,
    LIMIT_SWITCH_FAIL = 0x0008,
    HOMING_FAILED     = 0x0010,
    BROWNOUT          = 0x0040,
    WATCHDOG_RESET    = 0x0080,
    CONFIG_CORRUPT    = 0x0100
};

inline FaultCode operator|(FaultCode a, FaultCode b) {
    return static_cast<FaultCode>(static_cast<uint16_t>(a) | static_cast<uint16_t>(b));
}

inline FaultCode operator&(FaultCode a, FaultCode b) {
    return static_cast<FaultCode>(static_cast<uint16_t>(a) & static_cast<uint16_t>(b));
}

inline FaultCode& operator|=(FaultCode& a, FaultCode b) {
    a = a | b;
    return a;
}

inline bool hasFault(FaultCode flags, FaultCode test) {
    return (static_cast<uint16_t>(flags) & static_cast<uint16_t>(test)) != 0;
}

// --- Command Types ----------------------------------------------------------
enum class CommandType : uint8_t {
    NONE,
    CLOSE,       // Retract screen toward homing switch
    OPEN,        // Extend screen fully down
    STOP,
    HOME,
    RESET_FAULT,
    ENTER_SLEEP,
    WAKE
};

struct Command {
    CommandType type  = CommandType::NONE;
    int32_t     param = 0;       // Optional parameter (e.g., target position)
};

// --- Homing States ----------------------------------------------------------
enum class HomingState : uint8_t {
    UNINITIALIZED,
    SEEK_SWITCH,
    VERIFY_CONTACT,
    BACK_OFF,
    APPROACH_SLOW,
    SET_ZERO,
    COMPLETE,
    FAILED
};

// --- Enum-to-string converters (shared by REST API, MQTT, etc.) -------------

inline const char* systemStateToStr(SystemState s) {
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

inline const char* homingStateToStr(HomingState s) {
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

// --- Shared system snapshot, protected by mutex -----------------------------
struct SystemStatus {
    SystemState state        = SystemState::BOOT;
    HomingState homingState  = HomingState::UNINITIALIZED;
    FaultCode   faults       = FaultCode::NONE;
    int32_t     position     = 0;
    int32_t     targetPos    = 0;
    float       velocity     = 0.0f;       // Counts/sec
    bool        homed        = false;
    bool        limitSwitch  = false;      // true = switch activated
    bool        wifiConnected = false;
    uint32_t    uptimeMs     = 0;
};
