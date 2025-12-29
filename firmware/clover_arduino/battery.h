/*
 * ================================================================================
 *                              PROGETTO CLOVER
 *                        Battery Monitor Header File
 *                       LiPo 3S2P Battery Monitoring
 * ================================================================================
 *
 * Battery Specifications:
 *   - Type: LiPo 3S2P (3 series, 2 parallel)
 *   - Nominal Voltage: 11.1V
 *   - Max Voltage (charged): 12.6V
 *   - Min Voltage (cutoff): 9.0V
 *   - Low Warning: 10.0V
 *
 * Voltage Divider:
 *   - Required to reduce 12.6V max to <5V for Arduino ADC
 *   - Recommended: 10K + 3.3K resistor divider
 *   - Ratio = (10K + 3.3K) / 3.3K = 4.03
 *   - At 12.6V: Output = 12.6V / 4.03 = 3.13V (safe for Arduino)
 *
 * ================================================================================
 */

#ifndef BATTERY_H
#define BATTERY_H

#include <Arduino.h>
#include "config.h"

// ============================================================================
// Battery Status Enum
// ============================================================================

enum BatteryStatus {
    BATTERY_OK,
    BATTERY_LOW,
    BATTERY_CRITICAL,
    BATTERY_NOT_CONNECTED
};

// ============================================================================
// Battery Monitor Class
// ============================================================================

class BatteryMonitor {
public:
    // Constructor
    BatteryMonitor(uint8_t analogPin);

    // Initialization
    void begin();

    // Read voltage
    uint16_t readVoltage_mV();
    float readVoltage_V();

    // Status
    BatteryStatus getStatus();
    uint8_t getPercentage();

    // Averaging
    uint16_t readAveragedVoltage_mV(uint8_t samples = 10);

private:
    uint8_t _pin;
    uint16_t _lastVoltage_mV;

    uint16_t rawToVoltage(uint16_t rawADC);
};

// ============================================================================
// Implementation
// ============================================================================

BatteryMonitor::BatteryMonitor(uint8_t analogPin) {
    _pin = analogPin;
    _lastVoltage_mV = 0;
}

void BatteryMonitor::begin() {
    pinMode(_pin, INPUT);

    // Take initial reading with averaging
    _lastVoltage_mV = readAveragedVoltage_mV(10);
}

uint16_t BatteryMonitor::readVoltage_mV() {
    uint16_t rawADC = analogRead(_pin);
    _lastVoltage_mV = rawToVoltage(rawADC);
    return _lastVoltage_mV;
}

float BatteryMonitor::readVoltage_V() {
    return (float)readVoltage_mV() / 1000.0f;
}

uint16_t BatteryMonitor::readAveragedVoltage_mV(uint8_t samples) {
    uint32_t sum = 0;

    for (uint8_t i = 0; i < samples; i++) {
        sum += analogRead(_pin);
        delay(2);  // Short delay between samples
    }

    uint16_t avgRaw = sum / samples;
    _lastVoltage_mV = rawToVoltage(avgRaw);
    return _lastVoltage_mV;
}

BatteryStatus BatteryMonitor::getStatus() {
    if (_lastVoltage_mV < 1000) {
        return BATTERY_NOT_CONNECTED;
    }
    if (_lastVoltage_mV < BATTERY_CRITICAL_MV) {
        return BATTERY_CRITICAL;
    }
    if (_lastVoltage_mV < BATTERY_LOW_MV) {
        return BATTERY_LOW;
    }
    return BATTERY_OK;
}

uint8_t BatteryMonitor::getPercentage() {
    // Simple linear mapping (not accurate for LiPo discharge curve)
    // For better accuracy, implement a lookup table

    if (_lastVoltage_mV >= BATTERY_MAX_MV) {
        return 100;
    }
    if (_lastVoltage_mV <= BATTERY_CRITICAL_MV) {
        return 0;
    }

    // Linear interpolation between critical and max
    uint32_t range = BATTERY_MAX_MV - BATTERY_CRITICAL_MV;
    uint32_t current = _lastVoltage_mV - BATTERY_CRITICAL_MV;

    return (uint8_t)((current * 100) / range);
}

uint16_t BatteryMonitor::rawToVoltage(uint16_t rawADC) {
    // ADC value: 0-1023 maps to 0-5V (ADC_REF_MV)
    // Then multiply by voltage divider ratio

    // Calculate voltage at ADC pin (mV)
    uint32_t adcVoltage_mV = ((uint32_t)rawADC * ADC_REF_MV) / 1023;

    // Calculate actual battery voltage (mV)
    uint32_t batteryVoltage_mV = (uint32_t)(adcVoltage_mV * VOLTAGE_DIVIDER_RATIO);

    return (uint16_t)batteryVoltage_mV;
}

#endif // BATTERY_H
