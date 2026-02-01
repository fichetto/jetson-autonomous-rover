/*
 * ================================================================================
 *                              PROGETTO CLOVER
 *                       PID Controller Header File
 *              Integer-Only PID for Closed-Loop Velocity Control
 * ================================================================================
 *
 * Features:
 *   - Integer math only (no floating point for CPU efficiency)
 *   - Per-motor gain configuration
 *   - Anti-windup for integral term
 *   - Separate gains for each motor to compensate hardware differences
 *
 * Usage:
 *   1. Call pidInit() in setup()
 *   2. Set target RPM with pidSetTarget()
 *   3. Call pidCompute() every encoder update cycle
 *   4. Apply output to motor driver
 *
 * ================================================================================
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>
#include "config.h"

// ============================================================================
// PID State Structure
// ============================================================================

struct PIDState {
    int16_t setpoint;       // Target RPM (-200 to +200)
    int32_t integral;       // Accumulated error (32-bit to prevent overflow)
    int16_t lastError;      // Previous error (for derivative calculation)
    int16_t lastOutput;     // Last computed output (for debugging)
};

// ============================================================================
// PID Controller Class
// ============================================================================

class PIDController {
public:
    PIDController();

    // Initialization
    void begin();

    // Set target RPM for a motor
    void setTarget(uint8_t motorIndex, int16_t targetRPM);

    // Get current target
    int16_t getTarget(uint8_t motorIndex);

    // Compute PID output given measured RPM
    // Returns PWM value (-255 to +255)
    int16_t compute(uint8_t motorIndex, int16_t measuredRPM);

    // Reset a single motor's PID state
    void reset(uint8_t motorIndex);

    // Reset all motors
    void resetAll();

    // Get last output (for debugging/telemetry)
    int16_t getLastOutput(uint8_t motorIndex);

private:
    PIDState _state[4];

    // Per-motor gains (from config.h)
    static const int16_t _kp[4];
    static const int16_t _ki[4];
    static const int16_t _kd[4];
};

// ============================================================================
// Static Gain Arrays (defined from config.h macros)
// ============================================================================

const int16_t PIDController::_kp[4] = {PID_KP_FL, PID_KP_FR, PID_KP_RL, PID_KP_RR};
const int16_t PIDController::_ki[4] = {PID_KI_FL, PID_KI_FR, PID_KI_RL, PID_KI_RR};
const int16_t PIDController::_kd[4] = {PID_KD_FL, PID_KD_FR, PID_KD_RL, PID_KD_RR};

// ============================================================================
// Implementation
// ============================================================================

PIDController::PIDController() {
    // Initialize all states to zero
    for (uint8_t i = 0; i < 4; i++) {
        _state[i].setpoint = 0;
        _state[i].integral = 0;
        _state[i].lastError = 0;
        _state[i].lastOutput = 0;
    }
}

void PIDController::begin() {
    resetAll();
}

void PIDController::setTarget(uint8_t motorIndex, int16_t targetRPM) {
    if (motorIndex > 3) return;

    // Constrain target to valid range
    targetRPM = constrain(targetRPM, -PID_RPM_MAX, PID_RPM_MAX);
    _state[motorIndex].setpoint = targetRPM;
}

int16_t PIDController::getTarget(uint8_t motorIndex) {
    if (motorIndex > 3) return 0;
    return _state[motorIndex].setpoint;
}

int16_t PIDController::compute(uint8_t motorIndex, int16_t measuredRPM) {
    if (motorIndex > 3) return 0;

    PIDState* pid = &_state[motorIndex];

    // Calculate error
    int16_t error = pid->setpoint - measuredRPM;

    // Proportional term
    int32_t pTerm = (int32_t)_kp[motorIndex] * error;

    // Integral term with anti-windup
    pid->integral += error;
    pid->integral = constrain(pid->integral, -PID_INTEGRAL_MAX, PID_INTEGRAL_MAX);
    int32_t iTerm = (int32_t)_ki[motorIndex] * pid->integral;

    // Derivative term
    int16_t derivative = error - pid->lastError;
    pid->lastError = error;
    int32_t dTerm = (int32_t)_kd[motorIndex] * derivative;

    // Calculate output (divide by 100 because gains are scaled x100)
    int32_t output = (pTerm + iTerm + dTerm) / 100;

    // Constrain output to PWM range
    output = constrain(output, -PID_OUTPUT_MAX, PID_OUTPUT_MAX);

    // Store for debugging
    pid->lastOutput = (int16_t)output;

    return (int16_t)output;
}

void PIDController::reset(uint8_t motorIndex) {
    if (motorIndex > 3) return;

    _state[motorIndex].setpoint = 0;
    _state[motorIndex].integral = 0;
    _state[motorIndex].lastError = 0;
    _state[motorIndex].lastOutput = 0;
}

void PIDController::resetAll() {
    for (uint8_t i = 0; i < 4; i++) {
        reset(i);
    }
}

int16_t PIDController::getLastOutput(uint8_t motorIndex) {
    if (motorIndex > 3) return 0;
    return _state[motorIndex].lastOutput;
}

#endif // PID_CONTROLLER_H
