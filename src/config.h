#pragma once
// ============================================================================
// config.h — Central configuration for Rolling Projector Screen Controller
// All tunable parameters in one place. No magic numbers elsewhere.
// ============================================================================

#include <cstdint>

namespace cfg {

// --- Hardware Pin Assignments -----------------------------------------------
namespace pin {
    // BTS7960 Motor Driver
    constexpr uint8_t MOTOR_RPWM       = 25;   // Right PWM (motor forward)
    constexpr uint8_t MOTOR_LPWM       = 26;   // Left PWM (motor reverse)
    constexpr uint8_t MOTOR_R_EN       = 27;   // Right enable
    constexpr uint8_t MOTOR_L_EN       = 14;   // Left enable

    // Single-channel encoder (one data wire)
    constexpr uint8_t ENCODER_PIN      = 35;   // Input only GPIO

    // Limit Switch (normally closed, pulled high — LOW = pressed)
    constexpr uint8_t LIMIT_SWITCH     = 32;

    // INA226 I2C
    constexpr uint8_t I2C_SDA          = 21;
    constexpr uint8_t I2C_SCL          = 22;
}

// --- I2C Configuration ------------------------------------------------------
namespace i2c {
    constexpr uint8_t  INA226_ADDR     = 0x40;
    constexpr uint32_t CLOCK_HZ        = 400000;
}

// --- Motor / PWM Configuration ----------------------------------------------
namespace motor {
    constexpr uint32_t PWM_FREQ_HZ     = 20000;  // 20 kHz — above audible
    constexpr uint8_t  PWM_RESOLUTION  = 10;      // 10-bit: 0–1023
    constexpr uint16_t PWM_MAX         = (1 << PWM_RESOLUTION) - 1;
    constexpr uint8_t  PWM_CHANNEL_R   = 0;
    constexpr uint8_t  PWM_CHANNEL_L   = 1;
    constexpr uint16_t DEADBAND_PWM    = 30;      // Minimum PWM to overcome static friction
}

// --- Encoder Configuration --------------------------------------------------
// Single-channel, 1 CPR: 2 edges per revolution (toggles every 180°)
namespace encoder {
    constexpr int32_t  COUNTS_PER_REV  = 2;      // 2 edges per revolution
}

// --- PID Controller ---------------------------------------------------------
namespace pid {
    constexpr float    KP              = 2.0f;
    constexpr float    KI              = 0.5f;
    constexpr float    KD              = 0.05f;
    constexpr float    OUTPUT_MIN      = -1.0f;   // Normalized ±1.0
    constexpr float    OUTPUT_MAX      = 1.0f;
    constexpr float    INTEGRAL_MAX    = 0.8f;    // Anti-windup clamp
    constexpr float    DERIV_FILTER_ALPHA = 0.1f; // Low-pass on derivative (0–1, lower = smoother)
    constexpr float    DT_SEC          = 0.005f;  // 5 ms fixed timestep
}

// --- Motion Control ---------------------------------------------------------
// With 2 edges/rev, position values are in half-revolutions.
// Adjust these after measuring your actual screen travel.
namespace motion {
    constexpr int32_t  SOFT_LIMIT_MIN  = 0;           // Encoder counts
    constexpr int32_t  SOFT_LIMIT_MAX  = 1200;        // ~full screen travel
    constexpr int32_t  POSITION_TOLERANCE = 1;         // Counts — "close enough"
    constexpr uint32_t TIMEOUT_MS      = 15000;        // Max time for any move
    constexpr float    VELOCITY_THRESHOLD = 0.5f;      // Edges/sec — below = stalled
    constexpr int32_t  SCREEN_CLOSE_POS = 2;           // Near homing switch
    constexpr int32_t  SCREEN_OPEN_POS  = 1100;        // Fully extended
}

// --- Homing Configuration ---------------------------------------------------
namespace homing {
    constexpr float    SEEK_SPEED      = 0.5f;    // Normalized speed toward switch
    constexpr float    APPROACH_SPEED  = 0.15f;   // Slow approach speed
    constexpr int32_t  BACKOFF_COUNTS  = 8;       // ~4 revolutions backoff
    constexpr uint32_t PHASE_TIMEOUT_MS = 8000;   // Timeout per homing phase
    constexpr uint32_t DEBOUNCE_MS     = 50;      // Switch debounce
    constexpr uint32_t VERIFY_DWELL_MS = 200;     // Time to confirm switch contact
}

// --- Safety Thresholds ------------------------------------------------------
namespace safety {
    constexpr float    OVERCURRENT_MA  = 5000.0f;     // Motor stall / jam threshold
    constexpr float    WARNING_CURRENT_MA = 3500.0f;   // Pre-fault warning
    constexpr uint32_t STALL_TIME_MS   = 500;          // Duration before stall fault
    constexpr uint32_t CURRENT_SAMPLE_MS = 50;         // INA226 poll interval
    constexpr uint32_t MOTION_TIMEOUT_MS = 15000;      // Matches motion timeout
    constexpr uint8_t  OVERCURRENT_FILTER_COUNT = 3;   // Consecutive samples to confirm
}

// --- Network ----------------------------------------------------------------
namespace network {
    constexpr const char* WIFI_SSID    = "sharonmakover";
    constexpr const char* WIFI_PASS    = "0503527142";
    constexpr const char* HOSTNAME     = "projector-screen";
    constexpr uint16_t HTTP_PORT       = 80;
    constexpr uint32_t RECONNECT_INTERVAL_MS = 10000;
    constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 15000;
    constexpr uint32_t MAX_RECONNECT_ATTEMPTS = 10;
}

// --- Power Management -------------------------------------------------------
namespace power {
    constexpr uint32_t IDLE_TIMEOUT_MS       = 300000;       // 5 min before light sleep
    constexpr uint32_t SLEEP_WAKEUP_TIMER_US = 60000000;    // 60 sec periodic wake
    constexpr uint32_t DEEP_SLEEP_TIMEOUT_MS = 172800000UL; // 2 days → deep sleep (48h)
    constexpr uint32_t CPU_FREQ_ACTIVE_MHZ   = 240;
    constexpr uint32_t CPU_FREQ_IDLE_MHZ     = 80;
    constexpr uint32_t WIFI_IDLE_DISABLE_MS  = 120000;      // 2 min idle → WiFi off
}

// --- Buzzer / Audio Feedback ------------------------------------------------
namespace buzzer {
    constexpr uint8_t  PIN             = 13;      // GPIO for piezo buzzer
    constexpr uint8_t  PWM_CHANNEL     = 2;       // LEDC channel (0,1 used by motor)
    constexpr uint8_t  PWM_RESOLUTION  = 8;       // 8-bit duty resolution
}

// --- Storage / NVS ----------------------------------------------------------
namespace storage {
    constexpr const char* NVS_NAMESPACE     = "psc";          // Projector Screen Controller
    constexpr const char* KEY_POSITION      = "pos";
    constexpr const char* KEY_HOMED         = "homed";
    constexpr const char* KEY_CONFIG_CRC    = "cfg_crc";
    constexpr uint32_t    SAVE_INTERVAL_MS  = 5000;           // Min interval between NVS writes
}

// --- FreeRTOS Task Configuration --------------------------------------------
namespace task {
    constexpr uint32_t MOTION_PERIOD_MS     = 5;
    constexpr uint32_t SAFETY_PERIOD_MS     = 50;
    constexpr uint32_t NETWORK_PERIOD_MS    = 10;
    constexpr uint32_t APP_PERIOD_MS        = 50;
    constexpr uint32_t POWER_PERIOD_MS      = 1000;

    constexpr uint32_t MOTION_STACK         = 4096;
    constexpr uint32_t SAFETY_STACK         = 3072;
    constexpr uint32_t NETWORK_STACK        = 8192;
    constexpr uint32_t APP_STACK            = 4096;
    constexpr uint32_t POWER_STACK          = 3072;

    // Priorities (higher = more important, max configMAX_PRIORITIES-1)
    constexpr UBaseType_t MOTION_PRIO       = 5;
    constexpr UBaseType_t SAFETY_PRIO       = 5;
    constexpr UBaseType_t APP_PRIO          = 3;
    constexpr UBaseType_t NETWORK_PRIO      = 2;
    constexpr UBaseType_t POWER_PRIO        = 1;

    // Core affinity
    constexpr BaseType_t MOTION_CORE        = 1;   // Dedicated core for motion
    constexpr BaseType_t SAFETY_CORE        = 1;
    constexpr BaseType_t NETWORK_CORE       = 0;   // Protocol stack core
    constexpr BaseType_t APP_CORE           = 0;
    constexpr BaseType_t POWER_CORE         = 0;
}

// --- Watchdog ---------------------------------------------------------------
namespace watchdog {
    constexpr uint32_t TIMEOUT_SEC          = 10;
}

} // namespace cfg
