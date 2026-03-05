#pragma once
// ============================================================================
// persistent_storage.h — NVS-backed persistent storage with CRC validation
// ============================================================================

#include <Arduino.h>
#include <Preferences.h>
#include "config.h"

class PersistentStorage {
public:
    // Stored configuration structure — validated by CRC32
    struct StoredConfig {
        int32_t  position  = 0;
        uint8_t  homed     = 0;       // 1 = was homed before save
        uint32_t reserved  = 0;       // Future use
    };

    PersistentStorage() = default;

    bool begin() {
        if (!_prefs.begin(cfg::storage::NVS_NAMESPACE, false)) {
            return false;
        }
        _initialized = true;
        return true;
    }

    void end() {
        _prefs.end();
        _initialized = false;
    }

    // Save position with rate limiting
    bool savePosition(int32_t position, bool homed) {
        if (!_initialized) return false;

        uint32_t now = millis();
        if ((now - _lastSaveMs) < cfg::storage::SAVE_INTERVAL_MS) {
            return true; // Rate-limited, not an error
        }
        _lastSaveMs = now;

        StoredConfig sc;
        sc.position = position;
        sc.homed    = homed ? 1 : 0;

        // Write fields
        _prefs.putInt(cfg::storage::KEY_POSITION, sc.position);
        _prefs.putUChar(cfg::storage::KEY_HOMED, sc.homed);

        // Write CRC for validation
        uint32_t crc = computeCrc(sc);
        _prefs.putULong(cfg::storage::KEY_CONFIG_CRC, crc);

        return true;
    }

    // Load and validate stored config
    bool loadConfig(StoredConfig& out) {
        if (!_initialized) return false;

        out.position = _prefs.getInt(cfg::storage::KEY_POSITION, 0);
        out.homed    = _prefs.getUChar(cfg::storage::KEY_HOMED, 0);

        uint32_t storedCrc = _prefs.getULong(cfg::storage::KEY_CONFIG_CRC, 0);
        uint32_t computedCrc = computeCrc(out);

        if (storedCrc != computedCrc) {
            // CRC mismatch — config corrupt
            out = StoredConfig{}; // Reset to defaults
            return false;
        }

        return true;
    }

    // Force-write (ignore rate limit) — used before sleep
    bool forceSave(int32_t position, bool homed) {
        if (!_initialized) return false;
        _lastSaveMs = 0;
        return savePosition(position, homed);
    }

    void clearAll() {
        if (_initialized) {
            _prefs.clear();
        }
    }

private:
    Preferences _prefs;
    bool        _initialized = false;
    uint32_t    _lastSaveMs  = 0;

    // Simple CRC32 over StoredConfig bytes
    static uint32_t computeCrc(const StoredConfig& sc) {
        const uint8_t* data = reinterpret_cast<const uint8_t*>(&sc);
        size_t len = sizeof(StoredConfig);
        uint32_t crc = 0xFFFFFFFF;
        for (size_t i = 0; i < len; i++) {
            crc ^= data[i];
            for (int j = 0; j < 8; j++) {
                if (crc & 1) {
                    crc = (crc >> 1) ^ 0xEDB88320;
                } else {
                    crc >>= 1;
                }
            }
        }
        return crc ^ 0xFFFFFFFF;
    }
};
