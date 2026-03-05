// encoder.cpp — ISR implementation for single-channel pulse encoder
// ISR must be in .cpp for proper IRAM section placement on ESP32
#include <Arduino.h>
#include "motion/encoder.h"

// DRAM_ATTR ensures pointer is in data RAM, accessible from IRAM ISR
DRAM_ATTR SingleChannelEncoder* SingleChannelEncoder::_instance = nullptr;

void SingleChannelEncoder::begin(uint8_t pin) {
    _pin = pin;
    _count.store(0);
    _direction.store(0);
    _edges.store(0);

    pinMode(_pin, INPUT_PULLUP);

    _instance = this;

    attachInterrupt(digitalPinToInterrupt(_pin), isr, CHANGE);
}

void IRAM_ATTR SingleChannelEncoder::isr() {
    if (!_instance) return;
    _instance->_edges.fetch_add(1, std::memory_order_relaxed);
    int8_t dir = _instance->_direction.load(std::memory_order_relaxed);
    if (dir > 0) {
        _instance->_count.fetch_add(1, std::memory_order_relaxed);
    } else if (dir < 0) {
        _instance->_count.fetch_sub(1, std::memory_order_relaxed);
    }
}
