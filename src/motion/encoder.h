#pragma once
// ============================================================================
// encoder.h — Single-channel pulse encoder for ESP32
// Direction set externally from motor driver (single channel cannot
// determine rotation direction). Uses one GPIO interrupt on CHANGE.
// ============================================================================

#include <Arduino.h>
#include <atomic>
#include "config.h"

class SingleChannelEncoder {
public:
    SingleChannelEncoder() = default;

    void begin(uint8_t pin);

    // Atomic read — safe from any context
    int32_t read() const {
        return _count.load(std::memory_order_relaxed);
    }

    void write(int32_t value) {
        _count.store(value, std::memory_order_relaxed);
    }

    void reset() {
        _count.store(0, std::memory_order_relaxed);
    }

    // Set counting direction: +1 = forward, -1 = reverse, 0 = ignore edges
    void setDirection(int8_t dir) {
        _direction.store(dir, std::memory_order_relaxed);
    }

    // Raw edge count (always increments, ignores direction)
    uint32_t rawEdges() const {
        return _edges.load(std::memory_order_relaxed);
    }

    // Public for ISR access — do not call directly
    volatile std::atomic<int32_t>  _count{0};
    volatile std::atomic<int8_t>   _direction{0};
    volatile std::atomic<uint32_t> _edges{0};
    uint8_t _pin = 0;

    static SingleChannelEncoder* _instance;
    static void IRAM_ATTR isr();
};
