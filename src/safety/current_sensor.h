#pragma once
// ============================================================================
// current_sensor.h — INA226 current sensor abstraction
// Thread-safe I2C access with filtering
// ============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <INA226.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "config.h"

class CurrentSensor {
public:
    CurrentSensor() = default;

    bool begin() {
        _i2cMutex = xSemaphoreCreateMutex();
        if (!_i2cMutex) return false;

        Wire.begin(cfg::pin::I2C_SDA, cfg::pin::I2C_SCL);
        Wire.setClock(cfg::i2c::CLOCK_HZ);

        if (!_ina.begin()) {
            _fault = true;
            return false;
        }

        // Configure for continuous shunt+bus, 1.1ms conversion, 4x averaging
        _ina.setAverage(4);
        _ina.setBusVoltageConversionTime(5);  // index 5 = 1.1ms
        _ina.setShuntVoltageConversionTime(5);
        _ina.setMode(7);  // Shunt and bus, continuous
        _ina.setMaxCurrentShunt(10.0f, 0.01f); // 10A max, 10mΩ shunt

        _fault = false;
        return true;
    }

    // Read current in mA — call from SafetyTask only
    float readCurrentMa() {
        if (_fault) return 0.0f;

        float current = 0.0f;
        if (xSemaphoreTake(_i2cMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            current = _ina.getCurrent_mA();
            // Apply exponential moving average filter
            _filteredMa = _filteredMa + 0.3f * (fabsf(current) - _filteredMa);
            xSemaphoreGive(_i2cMutex);
        }
        return _filteredMa;
    }

    float lastFilteredMa() const { return _filteredMa; }
    bool hasFault() const { return _fault; }

private:
    INA226 _ina{cfg::i2c::INA226_ADDR};
    SemaphoreHandle_t _i2cMutex = nullptr;
    float _filteredMa = 0.0f;
    bool  _fault = false;
};
