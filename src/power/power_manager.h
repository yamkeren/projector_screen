#pragma once
// ============================================================================
// power_manager.h — Power management: light sleep, deep sleep, CPU scaling
//
// Power ladder:
//   Active (240 MHz) → CPU idle (80 MHz) → Light sleep (WiFi modem-sleep)
//   → Deep sleep (after 2 days, only EXT0 wakeup or manual reset)
// ============================================================================

#include <Arduino.h>
#include <esp_sleep.h>
#include <esp_pm.h>
#include <esp_wifi.h>
#include <driver/rtc_io.h>
#include "config.h"
#include "core/shared_state.h"
#include "network/wifi_manager.h"

// RTC_DATA_ATTR survives deep sleep — tracks cumulative idle time
RTC_DATA_ATTR static uint32_t rtc_cumulativeIdleSec = 0;
RTC_DATA_ATTR static bool     rtc_wasDeepSleep      = false;

class PowerManager {
public:
    // Callback type for pre-sleep preparation (stop motor, save NVS, etc.)
    using PreSleepCallback = void(*)();

    PowerManager(SharedState& shared, WifiManager& wifi)
        : _shared(shared), _wifi(wifi) {}

    /// Register a callback that runs before deep sleep (stops motor, saves NVS)
    void setPreSleepCallback(PreSleepCallback cb) { _preSleepCb = cb; }

    void begin() {
        // Configure wakeup sources for BOTH light and deep sleep:
        // Limit switch as ext0 wakeup (GPIO32 is RTC-capable, LOW = pressed)
        esp_sleep_enable_ext0_wakeup(static_cast<gpio_num_t>(cfg::pin::LIMIT_SWITCH), 0);
        // Timer wakeup for periodic light-sleep cycling
        esp_sleep_enable_timer_wakeup(cfg::power::SLEEP_WAKEUP_TIMER_US);

        _lastActivityMs = millis();
        _wifiLastActivityMs = millis();
        setCpuFrequency(cfg::power::CPU_FREQ_ACTIVE_MHZ);

        // Check if we just woke from deep sleep
        esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
        if (cause == ESP_SLEEP_WAKEUP_EXT0) {
            // Woke from deep sleep via limit switch (home button)
            rtc_wasDeepSleep = true;
            rtc_cumulativeIdleSec = 0; // Reset idle counter
            _shared.setEvent(EVT_WAKE_REQUEST);
            log_i("Woke from deep sleep via limit switch");
        } else if (cause == ESP_SLEEP_WAKEUP_UNDEFINED) {
            // Fresh power-on or manual reset
            rtc_wasDeepSleep = false;
            rtc_cumulativeIdleSec = 0;
            log_i("Fresh boot (power-on or reset)");
        } else {
            log_i("Woke from sleep, cause=%d", (int)cause);
        }
    }

    /// Was this boot a wakeup from deep sleep?
    bool wokeFromDeepSleep() const { return rtc_wasDeepSleep; }

    // Called from PowerTask periodically (~1 s)
    void update() {
        uint32_t now = millis();
        SystemState state = _shared.getState();

        // Activity tracking — any motion or homing resets all idle timers
        if (state == SystemState::MOVING || state == SystemState::HOMING) {
            _lastActivityMs = now;
            _wifiLastActivityMs = now;
            rtc_cumulativeIdleSec = 0;  // Reset deep-sleep counter
            ensureActive();
            return;
        }

        if (state == SystemState::FAULT) {
            // Keep system active during fault for diagnostics
            _lastActivityMs = now;
            return;
        }

        // --- CPU frequency scaling (30 s idle) ------------------------------
        if ((now - _lastActivityMs) > 30000 && !_cpuReduced) {
            setCpuFrequency(cfg::power::CPU_FREQ_IDLE_MHZ);
            _cpuReduced = true;
        }

        // --- Light sleep entry (5 min idle) ---------------------------------
        if ((now - _lastActivityMs) > cfg::power::IDLE_TIMEOUT_MS &&
            state == SystemState::IDLE) {
            enterLightSleep();
        }

        // --- Deep sleep entry (2 days cumulative idle) ----------------------
        uint32_t deepThresholdSec = cfg::power::DEEP_SLEEP_TIMEOUT_MS / 1000;
        if (rtc_cumulativeIdleSec >= deepThresholdSec &&
            state == SystemState::IDLE) {
            enterDeepSleep();
        }
    }

    void notifyActivity() {
        _lastActivityMs = millis();
        _wifiLastActivityMs = millis();
        rtc_cumulativeIdleSec = 0;  // Reset deep-sleep counter
        ensureActive();
    }

    // ---- Light Sleep -------------------------------------------------------
    // WiFi stays in modem-sleep so REST commands can still wake the CPU.
    // CPU halts, RAM preserved, ~15 mA.
    void enterLightSleep() {
        _shared.setState(SystemState::SLEEPING);

        // Keep WiFi in max modem-sleep — AP beacon wakes the radio
        esp_wifi_set_ps(WIFI_PS_MAX_MODEM);

        // Reduce CPU before sleep
        setCpuFrequency(cfg::power::CPU_FREQ_IDLE_MHZ);

        // Enter light sleep — blocks until wakeup
        esp_light_sleep_start();

        // --- Wakeup path (resumes here) ---
        onLightWakeup();
    }

    // ---- Deep Sleep --------------------------------------------------------
    // Everything off except RTC. Only wakes from:
    //   1. Limit switch (EXT0 on GPIO32)  — "home button"
    //   2. Manual reset button on the ESP32
    // RAM is lost, setup() runs again. ~10 µA.
    void enterDeepSleep() {
        log_i("Entering deep sleep after %lu sec idle", rtc_cumulativeIdleSec);

        _shared.setState(SystemState::SLEEPING);

        // 1. Stop motor explicitly and save NVS via pre-sleep callback
        if (_preSleepCb) {
            _preSleepCb();
        }

        // 2. Disable hardware watchdog to prevent reset during shutdown
        esp_task_wdt_delete(NULL);

        // 3. Wait for NVS write to complete (AppStateTask checks EVT_WAKE_REQUEST)
        vTaskDelay(pdMS_TO_TICKS(200));

        // 4. Disable WiFi fully
        if (_wifi.isConnected()) {
            _wifi.disable();
            _wifiDisabled = true;
        }

        // 5. Configure deep-sleep wakeup: only EXT0 (limit switch)
        //    Timer wakeup is NOT enabled — stay in deep sleep until
        //    physically woken by switch press or manual reset.
        esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
        esp_sleep_enable_ext0_wakeup(static_cast<gpio_num_t>(cfg::pin::LIMIT_SWITCH), 0);

        // 6. Hold RTC GPIO state so the pull-up remains active
        rtc_gpio_pullup_en(static_cast<gpio_num_t>(cfg::pin::LIMIT_SWITCH));
        rtc_gpio_pulldown_dis(static_cast<gpio_num_t>(cfg::pin::LIMIT_SWITCH));

        // 7. Enter deep sleep — does NOT return, next execution starts at setup()
        esp_deep_sleep_start();
    }

    bool isWifiDisabled() const { return _wifiDisabled; }

private:
    SharedState& _shared;
    WifiManager& _wifi;
    PreSleepCallback _preSleepCb = nullptr;

    uint32_t _lastActivityMs     = 0;
    uint32_t _wifiLastActivityMs = 0;
    bool     _cpuReduced         = false;
    bool     _wifiDisabled       = false;

    void onLightWakeup() {
        uint32_t now = millis();
        _lastActivityMs = now;
        _wifiLastActivityMs = now;

        setCpuFrequency(cfg::power::CPU_FREQ_ACTIVE_MHZ);
        _cpuReduced = false;

        // Restore WiFi to active mode
        esp_wifi_set_ps(WIFI_PS_NONE);

        // Check wakeup reason BEFORE accumulating idle time
        esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

        if (cause == ESP_SLEEP_WAKEUP_EXT0) {
            // Limit switch triggered during light sleep — real activity
            rtc_cumulativeIdleSec = 0;
            _shared.setEvent(EVT_WAKE_REQUEST);
        } else if (cause == ESP_SLEEP_WAKEUP_TIMER) {
            // Periodic timer — accumulate toward deep-sleep threshold
            rtc_cumulativeIdleSec += cfg::power::SLEEP_WAKEUP_TIMER_US / 1000000;
        }
        // Other causes (e.g., WiFi waking CPU via modem-sleep) do NOT
        // accumulate idle time — the device is being actively used.

        _shared.setState(SystemState::IDLE);
    }

    void setCpuFrequency(uint32_t mhz) {
        setCpuFrequencyMhz(mhz);
    }

    void ensureActive() {
        if (_cpuReduced) {
            setCpuFrequency(cfg::power::CPU_FREQ_ACTIVE_MHZ);
            _cpuReduced = false;
        }
        // Restore WiFi to full active if it was in modem-sleep
        esp_wifi_set_ps(WIFI_PS_NONE);

        if (_wifiDisabled) {
            _wifi.enable();
            _wifiDisabled = false;
        }
    }
};
