#pragma once
// ============================================================================
// shared_state.h — Thread-safe shared state container
// Single point of truth for inter-task communication
// ============================================================================

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include <freertos/event_groups.h>
#include "system_types.h"

// Event group bits
constexpr EventBits_t EVT_COMMAND_READY   = (1 << 0);
constexpr EventBits_t EVT_FAULT_OCCURRED  = (1 << 1);
constexpr EventBits_t EVT_MOTION_COMPLETE = (1 << 2);
constexpr EventBits_t EVT_HOMING_DONE     = (1 << 3);
constexpr EventBits_t EVT_WAKE_REQUEST    = (1 << 4);
constexpr EventBits_t EVT_WIFI_CONNECTED  = (1 << 5);

class SharedState {
public:
    SharedState() {
        _mutex = xSemaphoreCreateMutex();
        configASSERT(_mutex != nullptr);

        _cmdQueue = xQueueCreate(8, sizeof(Command));
        configASSERT(_cmdQueue != nullptr);

        _events = xEventGroupCreate();
        configASSERT(_events != nullptr);
    }

    ~SharedState() {
        if (_mutex)    vSemaphoreDelete(_mutex);
        if (_cmdQueue) vQueueDelete(_cmdQueue);
        if (_events)   vEventGroupDelete(_events);
    }

    // Non-copyable
    SharedState(const SharedState&) = delete;
    SharedState& operator=(const SharedState&) = delete;

    // --- Status access (mutex-protected) ------------------------------------

    SystemStatus getStatus() const {
        SystemStatus copy;
        if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            copy = _status;
            xSemaphoreGive(_mutex);
        }
        return copy;
    }

    void updateStatus(const SystemStatus& s) {
        if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            _status = s;
            xSemaphoreGive(_mutex);
        }
    }

    // Selective field updates to minimize lock duration
    void setPosition(int32_t pos) {
        lock([&]() { _status.position = pos; });
    }

    void setTargetPosition(int32_t pos) {
        lock([&]() { _status.targetPos = pos; });
    }

    void setVelocity(float vel) {
        lock([&]() { _status.velocity = vel; });
    }

    void setState(SystemState st) {
        lock([&]() { _status.state = st; });
    }

    void setHomingState(HomingState hs) {
        lock([&]() { _status.homingState = hs; });
    }

    void setFaults(FaultCode f) {
        lock([&]() { _status.faults = f; });
    }

    void addFault(FaultCode f) {
        lock([&]() { _status.faults |= f; });
    }

    void setHomed(bool h) {
        lock([&]() { _status.homed = h; });
    }

    void setLimitSwitch(bool ls) {
        lock([&]() { _status.limitSwitch = ls; });
    }

    void setWifiConnected(bool c) {
        lock([&]() { _status.wifiConnected = c; });
    }

    void setUptimeMs(uint32_t ms) {
        lock([&]() { _status.uptimeMs = ms; });
    }

    SystemState getState() const {
        return lockRead([&]() { return _status.state; });
    }

    FaultCode getFaults() const {
        return lockRead([&]() { return _status.faults; });
    }

    bool isHomed() const {
        return lockRead([&]() { return _status.homed; });
    }

    // --- Command Queue ------------------------------------------------------

    bool sendCommand(const Command& cmd, TickType_t wait = pdMS_TO_TICKS(50)) {
        if (xQueueSend(_cmdQueue, &cmd, wait) == pdTRUE) {
            xEventGroupSetBits(_events, EVT_COMMAND_READY);
            return true;
        }
        return false;
    }

    bool receiveCommand(Command& cmd, TickType_t wait = 0) {
        return xQueueReceive(_cmdQueue, &cmd, wait) == pdTRUE;
    }

    // --- Event Group --------------------------------------------------------

    EventGroupHandle_t events() const { return _events; }

    void setEvent(EventBits_t bits) {
        xEventGroupSetBits(_events, bits);
    }

    void clearEvent(EventBits_t bits) {
        xEventGroupClearBits(_events, bits);
    }

    EventBits_t waitEvent(EventBits_t bits, bool clearOnExit, TickType_t wait) {
        return xEventGroupWaitBits(_events, bits, clearOnExit ? pdTRUE : pdFALSE,
                                   pdFALSE, wait);
    }

private:
    mutable SemaphoreHandle_t _mutex       = nullptr;
    QueueHandle_t             _cmdQueue    = nullptr;
    EventGroupHandle_t        _events      = nullptr;
    SystemStatus              _status      = {};

    template<typename Fn>
    void lock(Fn fn) {
        if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            fn();
            xSemaphoreGive(_mutex);
        }
    }

    template<typename Fn>
    auto lockRead(Fn fn) const -> decltype(fn()) {
        decltype(fn()) result{};
        if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            result = fn();
            xSemaphoreGive(_mutex);
        }
        return result;
    }
};
