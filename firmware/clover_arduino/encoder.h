/*
 * ================================================================================
 *                              PROGETTO CLOVER
 *                          Encoder Handler Header
 *                   Hall Effect Encoder Reading (JGB 520)
 * ================================================================================
 *
 * Encoder Configuration:
 *   - M1, M2: Hardware interrupts (D2/INT0, D3/INT1)
 *   - M3, M4: Polling mode (D4, D5)
 *
 * JGB 520 Encoder Specs:
 *   - CPR (Counts Per Revolution): 11 (motor shaft)
 *   - Gear Ratio: 30:1
 *   - Effective CPR at wheel: 330 ticks/revolution
 *
 * ================================================================================
 */

#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>
#include "config.h"

// ============================================================================
// Encoder Data Structure
// ============================================================================

struct EncoderData {
    volatile long count;            // Tick count (can be negative)
    volatile uint8_t lastState;     // Last pin state (for polling)
    long lastCount;                 // Previous count (for RPM calc)
    unsigned long lastTime;         // Timestamp for RPM calculation
    int16_t rpm;                    // Calculated RPM
};

// ============================================================================
// Global Encoder Data (for ISR access)
// ============================================================================

volatile EncoderData encoderData[4];

// ============================================================================
// Interrupt Service Routines
// ============================================================================

void encoderISR_M1() {
    encoderData[0].count++;
}

void encoderISR_M2() {
    encoderData[1].count++;
}

// ============================================================================
// Encoder Manager Class
// ============================================================================

class EncoderManager {
public:
    EncoderManager();

    // Initialization
    void begin();

    // Update (call regularly for polling encoders)
    void update();

    // Get encoder data
    long getCount(uint8_t encoderIndex);
    int16_t getRPM(uint8_t encoderIndex);

    // Reset functions
    void resetCount(uint8_t encoderIndex);
    void resetAll();

private:
    uint8_t _pins[4];
    unsigned long _lastUpdateTime;

    void updatePollingEncoder(uint8_t index, uint8_t pin);
    void calculateRPM(uint8_t index);
};

// ============================================================================
// Implementation
// ============================================================================

EncoderManager::EncoderManager() {
    _pins[0] = ENCODER_M1_PIN;
    _pins[1] = ENCODER_M2_PIN;
    _pins[2] = ENCODER_M3_PIN;
    _pins[3] = ENCODER_M4_PIN;
    _lastUpdateTime = 0;
}

void EncoderManager::begin() {
    // Initialize encoder data structures
    for (uint8_t i = 0; i < 4; i++) {
        encoderData[i].count = 0;
        encoderData[i].lastState = 0;
        encoderData[i].lastCount = 0;
        encoderData[i].lastTime = millis();
        encoderData[i].rpm = 0;
    }

    // Configure pins
    pinMode(ENCODER_M1_PIN, INPUT_PULLUP);
    pinMode(ENCODER_M2_PIN, INPUT_PULLUP);
    pinMode(ENCODER_M3_PIN, INPUT_PULLUP);
    pinMode(ENCODER_M4_PIN, INPUT_PULLUP);

    // Read initial states for polling encoders
    encoderData[2].lastState = digitalRead(ENCODER_M3_PIN);
    encoderData[3].lastState = digitalRead(ENCODER_M4_PIN);

    // Attach interrupts for M1 and M2
    attachInterrupt(digitalPinToInterrupt(ENCODER_M1_PIN), encoderISR_M1, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_M2_PIN), encoderISR_M2, RISING);

    _lastUpdateTime = millis();
}

void EncoderManager::update() {
    unsigned long currentTime = millis();

    // Update polling encoders (M3, M4)
    updatePollingEncoder(2, ENCODER_M3_PIN);
    updatePollingEncoder(3, ENCODER_M4_PIN);

    // Calculate RPM for all encoders (at defined interval)
    if (currentTime - _lastUpdateTime >= ENCODER_UPDATE_INTERVAL) {
        for (uint8_t i = 0; i < 4; i++) {
            calculateRPM(i);
        }
        _lastUpdateTime = currentTime;
    }
}

void EncoderManager::updatePollingEncoder(uint8_t index, uint8_t pin) {
    uint8_t currentState = digitalRead(pin);

    // Detect rising edge
    if (currentState == HIGH && encoderData[index].lastState == LOW) {
        encoderData[index].count++;
    }

    encoderData[index].lastState = currentState;
}

void EncoderManager::calculateRPM(uint8_t index) {
    unsigned long currentTime = millis();
    unsigned long deltaTime = currentTime - encoderData[index].lastTime;

    if (deltaTime == 0) return;

    // Get current count (disable interrupts briefly for consistency)
    noInterrupts();
    long currentCount = encoderData[index].count;
    interrupts();

    // Calculate ticks since last measurement
    long deltaTicks = currentCount - encoderData[index].lastCount;

    // Calculate RPM
    // RPM = (ticks / TICKS_PER_REV) * (60000 / deltaTime_ms)
    // Simplified: RPM = (ticks * 60000) / (TICKS_PER_REV * deltaTime_ms)

    float rpm = ((float)deltaTicks * 60000.0f) / ((float)TICKS_PER_REV * (float)deltaTime);

    // Store RPM (signed to indicate direction)
    encoderData[index].rpm = (int16_t)rpm;

    // Update for next calculation
    encoderData[index].lastCount = currentCount;
    encoderData[index].lastTime = currentTime;
}

long EncoderManager::getCount(uint8_t encoderIndex) {
    if (encoderIndex > 3) return 0;

    // Disable interrupts briefly for consistent read
    noInterrupts();
    long count = encoderData[encoderIndex].count;
    interrupts();

    return count;
}

int16_t EncoderManager::getRPM(uint8_t encoderIndex) {
    if (encoderIndex > 3) return 0;
    return encoderData[encoderIndex].rpm;
}

void EncoderManager::resetCount(uint8_t encoderIndex) {
    if (encoderIndex > 3) return;

    noInterrupts();
    encoderData[encoderIndex].count = 0;
    encoderData[encoderIndex].lastCount = 0;
    interrupts();
}

void EncoderManager::resetAll() {
    for (uint8_t i = 0; i < 4; i++) {
        resetCount(i);
    }
}

#endif // ENCODER_H
