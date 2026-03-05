#pragma once
// ============================================================================
// buzzer.h — Non-blocking piezo buzzer driver with queued sound patterns
//
// Architecture:
//   - Hardware:  LEDC PWM channel for square-wave tone generation
//   - Patterns:  Array of {freq, duration} steps — fully data-driven
//   - Playback:  Finite state machine ticked from a FreeRTOS task loop
//   - Queueing:  FreeRTOS queue for thread-safe, non-blocking requests
//
// LEDC channel 2 (timer 1) is used to avoid conflict with motor PWM
// which occupies channels 0,1 (timer 0).
// ============================================================================

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "config.h"

class Buzzer {
public:
    Buzzer() = default;

    ~Buzzer() {
        if (_queue) vQueueDelete(_queue);
    }

    // Non-copyable
    Buzzer(const Buzzer&) = delete;
    Buzzer& operator=(const Buzzer&) = delete;

    // --- Lifecycle ----------------------------------------------------------

    /// Initialize LEDC hardware and internal queue. Call once from setup().
    void begin() {
        _queue = xQueueCreate(QUEUE_SIZE, sizeof(SoundId));
        configASSERT(_queue != nullptr);

        ledcSetup(cfg::buzzer::PWM_CHANNEL, 1000, cfg::buzzer::PWM_RESOLUTION);
        ledcAttachPin(cfg::buzzer::PIN, cfg::buzzer::PWM_CHANNEL);
        ledcWrite(cfg::buzzer::PWM_CHANNEL, 0);

        _active   = nullptr;
        _stepIdx  = 0;
        log_i("Buzzer ready on GPIO %d (LEDC ch%d)", cfg::buzzer::PIN, cfg::buzzer::PWM_CHANNEL);
    }

    /// Tick the playback FSM. Call from a task loop at ≤50 ms period.
    void update() {
        // If idle, try to dequeue the next sound request
        if (!_active) {
            SoundId id;
            if (xQueueReceive(_queue, &id, 0) == pdTRUE) {
                startPattern(id);
            }
            return;
        }

        // Currently playing — check if the current step has elapsed
        uint32_t now = millis();
        if ((now - _stepStartMs) >= _active->steps[_stepIdx].durationMs) {
            _stepIdx++;
            if (_stepIdx >= _active->count) {
                // Pattern complete
                toneOff();
                _active = nullptr;
                return;
            }
            applyCurrentStep();
        }
    }

    // --- Sound triggers (thread-safe, non-blocking) -------------------------
    //
    // Safe to call from any FreeRTOS task or ISR context.
    // Sounds are queued; if the queue is full the request is silently dropped.

    void playWifiConnecting()  { enqueue(SoundId::WIFI_CONNECTING); }
    void playWifiSuccess()     { enqueue(SoundId::WIFI_SUCCESS); }
    void playWifiFailure()     { enqueue(SoundId::WIFI_FAILURE); }
    void playHoming()          { enqueue(SoundId::HOMING); }
    void playScreenDown()      { enqueue(SoundId::SCREEN_DOWN); }
    void playScreenClosing()   { enqueue(SoundId::SCREEN_CLOSING); }

    // --- Control ------------------------------------------------------------

    /// Immediately silence and flush the queue.
    void stop() {
        toneOff();
        _active = nullptr;
        SoundId discard;
        while (xQueueReceive(_queue, &discard, 0) == pdTRUE) {}
    }

    bool isPlaying() const { return _active != nullptr; }

    /// Mute suppresses tone output but patterns still advance timing.
    void setMute(bool m) {
        _muted = m;
        if (m) toneOff();
    }

    bool isMuted() const { return _muted; }

private:
    // ---- Internal types ----------------------------------------------------

    /// A single tone step: frequency in Hz (0 = silence/rest) + duration.
    struct ToneStep {
        uint16_t freqHz;
        uint16_t durationMs;
    };

    /// Identifies a sound pattern in the lookup table.
    enum class SoundId : uint8_t {
        NONE = 0,
        WIFI_CONNECTING,
        WIFI_SUCCESS,
        WIFI_FAILURE,
        HOMING,
        SCREEN_DOWN,
        SCREEN_CLOSING,
        _COUNT              // sentinel — must be last
    };

    static constexpr uint8_t MAX_STEPS  = 8;
    static constexpr uint8_t QUEUE_SIZE = 4;

    /// Fixed-size pattern: up to MAX_STEPS tone steps.
    struct SoundPattern {
        ToneStep steps[MAX_STEPS];
        uint8_t  count;         // How many steps are valid
    };

    // ---- Pattern table (defined in buzzer.cpp) -----------------------------
    static const SoundPattern _patterns[];

    // ---- Playback state ----------------------------------------------------
    const SoundPattern* _active     = nullptr;
    uint8_t             _stepIdx    = 0;
    uint32_t            _stepStartMs = 0;
    bool                _muted      = false;

    // ---- Sound request queue -----------------------------------------------
    QueueHandle_t _queue = nullptr;

    // ---- Private helpers ---------------------------------------------------

    void enqueue(SoundId id) {
        if (_queue) {
            // Non-blocking send — drop if full (acceptable for audio feedback)
            xQueueSend(_queue, &id, 0);
        }
    }

    void startPattern(SoundId id) {
        uint8_t idx = static_cast<uint8_t>(id) - 1;   // SoundId enum starts at 1
        if (idx >= (static_cast<uint8_t>(SoundId::_COUNT) - 1)) return;

        _active  = &_patterns[idx];
        _stepIdx = 0;
        applyCurrentStep();
    }

    void applyCurrentStep() {
        _stepStartMs = millis();
        const ToneStep& step = _active->steps[_stepIdx];
        if (step.freqHz == 0 || _muted) {
            toneOff();
        } else {
            toneOn(step.freqHz);
        }
    }

    void toneOn(uint16_t freqHz) {
        ledcWriteTone(cfg::buzzer::PWM_CHANNEL, static_cast<double>(freqHz));
    }

    void toneOff() {
        ledcWriteTone(cfg::buzzer::PWM_CHANNEL, 0);
    }
};
