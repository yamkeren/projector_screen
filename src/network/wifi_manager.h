#pragma once
// ============================================================================
// wifi_manager.h — WiFi management with captive portal provisioning
// Uses WiFiManager library for initial setup, simple reconnect at runtime
// ============================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include "config.h"
#include "core/shared_state.h"
#include "audio/buzzer.h"

class WifiManager {
public:
    enum class State : uint8_t {
        DISCONNECTED,
        CONNECTED,
        OFF
    };

    explicit WifiManager(SharedState& shared) : _shared(shared) {}

    // Blocking — call from setup() before FreeRTOS tasks.
    // Opens a captive-portal AP if no saved credentials are found.
    bool begin() {
        WiFiManager wm;
        wm.setHostname(cfg::network::HOSTNAME);
        wm.setConnectTimeout(cfg::network::WIFI_CONNECT_TIMEOUT_MS / 1000);
        wm.setConfigPortalTimeout(cfg::network::PORTAL_TIMEOUT_SEC);

        if (_buzzer) _buzzer->playWifiConnecting();

        bool ok = wm.autoConnect(cfg::network::AP_NAME);
        if (ok) {
            _state = State::CONNECTED;
            _shared.setWifiConnected(true);
            _shared.setEvent(EVT_WIFI_CONNECTED);
            if (_buzzer) _buzzer->playWifiSuccess();
        } else {
            _state = State::DISCONNECTED;
            if (_buzzer) _buzzer->playWifiFailure();
        }
        return ok;
    }

    // Non-blocking — call from NetworkTask
    void update() {
        if (_state == State::OFF) return;

        if (WiFi.status() == WL_CONNECTED) {
            if (_state != State::CONNECTED) {
                _state = State::CONNECTED;
                _attempts = 0;
                _shared.setWifiConnected(true);
                _shared.setEvent(EVT_WIFI_CONNECTED);
                if (_buzzer) _buzzer->playWifiSuccess();
            }
        } else {
            if (_state == State::CONNECTED) {
                _state = State::DISCONNECTED;
                _shared.setWifiConnected(false);
                _shared.clearEvent(EVT_WIFI_CONNECTED);
            }
            tryReconnect();
        }
    }

    void enable() {
        if (_state == State::OFF) {
            WiFi.mode(WIFI_STA);
            _state = State::DISCONNECTED;
            _attempts = 0;
        }
    }

    void disable() {
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
        _state = State::OFF;
        _shared.setWifiConnected(false);
    }

    State     state() const { return _state; }
    bool      isConnected() const { return _state == State::CONNECTED; }
    IPAddress localIP() const { return WiFi.localIP(); }

    void setBuzzer(Buzzer* bz) { _buzzer = bz; }

private:
    SharedState& _shared;
    Buzzer*  _buzzer        = nullptr;
    State    _state         = State::DISCONNECTED;
    uint32_t _lastAttemptMs = 0;
    uint32_t _attempts      = 0;

    void tryReconnect() {
        uint32_t now = millis();
        if ((now - _lastAttemptMs) < cfg::network::RECONNECT_INTERVAL_MS) return;
        _lastAttemptMs = now;

        if (_attempts >= cfg::network::MAX_RECONNECT_ATTEMPTS) {
            _attempts = 0;
            return;
        }

        WiFi.reconnect();
        _attempts++;
    }
};
