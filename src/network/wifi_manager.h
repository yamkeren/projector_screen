#pragma once
// ============================================================================
// wifi_manager.h — Non-blocking WiFi manager with reconnect logic
// ============================================================================

#include <Arduino.h>
#include <WiFi.h>
#include "config.h"
#include "core/shared_state.h"
#include "audio/buzzer.h"

class WifiManager {
public:
    enum class State : uint8_t {
        DISCONNECTED,
        CONNECTING,
        CONNECTED,
        RECONNECTING,
        OFF
    };

    explicit WifiManager(SharedState& shared) : _shared(shared) {}

    void begin() {
        WiFi.mode(WIFI_STA);
        WiFi.setHostname(cfg::network::HOSTNAME);
        WiFi.setAutoReconnect(false); // We manage reconnection
        WiFi.setSleep(false);         // Disable modem sleep initially

        _state = State::DISCONNECTED;
        _lastAttemptMs = 0;
        _attempts = 0;
    }

    // Non-blocking update — call from NetworkTask
    void update() {
        switch (_state) {
            case State::OFF:
                _shared.setWifiConnected(false);
                return;

            case State::DISCONNECTED:
                startConnect();
                break;

            case State::CONNECTING:
            case State::RECONNECTING:
                checkConnection();
                break;

            case State::CONNECTED:
                if (WiFi.status() != WL_CONNECTED) {
                    _state = State::RECONNECTING;
                    _lastAttemptMs = millis();
                    _attempts = 0;
                    _shared.setWifiConnected(false);
                    _shared.clearEvent(EVT_WIFI_CONNECTED);
                }
                break;
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

    State state() const { return _state; }
    bool  isConnected() const { return _state == State::CONNECTED; }
    IPAddress localIP() const { return WiFi.localIP(); }

    void setBuzzer(Buzzer* bz) { _buzzer = bz; }

private:
    SharedState& _shared;
    Buzzer*  _buzzer         = nullptr;
    State    _state          = State::DISCONNECTED;
    uint32_t _lastAttemptMs  = 0;
    uint32_t _connectStartMs = 0;
    uint32_t _attempts       = 0;

    void startConnect() {
        if ((millis() - _lastAttemptMs) < cfg::network::RECONNECT_INTERVAL_MS &&
            _lastAttemptMs != 0) {
            return; // Rate-limit connection attempts
        }

        if (_attempts >= cfg::network::MAX_RECONNECT_ATTEMPTS) {
            // Back off — stay disconnected, try again later
            _lastAttemptMs = millis();
            _attempts = 0;
            return;
        }

        WiFi.begin(cfg::network::WIFI_SSID, cfg::network::WIFI_PASS);
        _connectStartMs = millis();
        _lastAttemptMs = millis();
        _state = State::CONNECTING;
        _attempts++;

        if (_buzzer && _attempts == 1) _buzzer->playWifiConnecting();
    }

    void checkConnection() {
        if (WiFi.status() == WL_CONNECTED) {
            _state = State::CONNECTED;
            _attempts = 0;
            _shared.setWifiConnected(true);
            _shared.setEvent(EVT_WIFI_CONNECTED);
            if (_buzzer) _buzzer->playWifiSuccess();
            return;
        }

        // Timeout on this attempt
        if ((millis() - _connectStartMs) > cfg::network::WIFI_CONNECT_TIMEOUT_MS) {
            WiFi.disconnect(true);
            _state = State::DISCONNECTED;
            if (_buzzer) _buzzer->playWifiFailure();
        }
    }
};
