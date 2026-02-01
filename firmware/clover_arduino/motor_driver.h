/*
 * ================================================================================
 *                              PROGETTO CLOVER
 *                         Motor Driver Header File
 *                    PCA9685 + HR8833 Motor Control
 * ================================================================================
 */

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include "config.h"

// ============================================================================
// Motor Channel Structure
// ============================================================================

struct MotorChannel {
    uint8_t in1_channel;    // PCA9685 channel for IN1
    uint8_t in2_channel;    // PCA9685 channel for IN2
    bool inverted;          // Motor direction inversion
    int16_t currentSpeed;   // Current speed (-255 to +255)
    int8_t pwmOffset;       // PWM calibration offset (-50 to +50)
    uint8_t pwmScale;       // PWM scale factor (percentage, 100 = normal)
};

// ============================================================================
// Motor Driver Class
// ============================================================================

class MotorDriver {
public:
    // Constructor
    MotorDriver(Adafruit_PWMServoDriver* pwmDriver);

    // Initialization
    void begin();

    // Motor control
    void setMotorSpeed(uint8_t motorIndex, int16_t speed);
    void setAllMotorSpeeds(int16_t speedFL, int16_t speedFR,
                           int16_t speedRL, int16_t speedRR);

    // Stop functions
    void stopMotor(uint8_t motorIndex);
    void stopAll();
    void emergencyStop();

    // Brake functions
    void brakeMotor(uint8_t motorIndex);
    void brakeAll();

    // Status
    int16_t getMotorSpeed(uint8_t motorIndex);
    bool isEmergencyStopped();

private:
    Adafruit_PWMServoDriver* _pwm;
    MotorChannel _motors[4];
    bool _emergencyStop;

    // Internal functions
    void applyMotorOutput(uint8_t motorIndex, int16_t speed);
    uint16_t speedToPWM(int16_t speed);
};

// ============================================================================
// Implementation
// ============================================================================

MotorDriver::MotorDriver(Adafruit_PWMServoDriver* pwmDriver) {
    _pwm = pwmDriver;
    _emergencyStop = false;

    // Initialize motor channel mappings
    // Motor FL (Front Left)
    _motors[MOTOR_FL].in1_channel = PWM_FL_IN1;
    _motors[MOTOR_FL].in2_channel = PWM_FL_IN2;
    _motors[MOTOR_FL].inverted = INVERT_FL;
    _motors[MOTOR_FL].currentSpeed = 0;
    _motors[MOTOR_FL].pwmOffset = PWM_OFFSET_FL;
    _motors[MOTOR_FL].pwmScale = PWM_SCALE_FL;

    // Motor FR (Front Right)
    _motors[MOTOR_FR].in1_channel = PWM_FR_IN1;
    _motors[MOTOR_FR].in2_channel = PWM_FR_IN2;
    _motors[MOTOR_FR].inverted = INVERT_FR;
    _motors[MOTOR_FR].currentSpeed = 0;
    _motors[MOTOR_FR].pwmOffset = PWM_OFFSET_FR;
    _motors[MOTOR_FR].pwmScale = PWM_SCALE_FR;

    // Motor RL (Rear Left)
    _motors[MOTOR_RL].in1_channel = PWM_RL_IN1;
    _motors[MOTOR_RL].in2_channel = PWM_RL_IN2;
    _motors[MOTOR_RL].inverted = INVERT_RL;
    _motors[MOTOR_RL].currentSpeed = 0;
    _motors[MOTOR_RL].pwmOffset = PWM_OFFSET_RL;
    _motors[MOTOR_RL].pwmScale = PWM_SCALE_RL;

    // Motor RR (Rear Right)
    _motors[MOTOR_RR].in1_channel = PWM_RR_IN1;
    _motors[MOTOR_RR].in2_channel = PWM_RR_IN2;
    _motors[MOTOR_RR].inverted = INVERT_RR;
    _motors[MOTOR_RR].currentSpeed = 0;
    _motors[MOTOR_RR].pwmOffset = PWM_OFFSET_RR;
    _motors[MOTOR_RR].pwmScale = PWM_SCALE_RR;
}

void MotorDriver::begin() {
    // Stop all motors on init
    stopAll();
    _emergencyStop = false;
}

void MotorDriver::setMotorSpeed(uint8_t motorIndex, int16_t speed) {
    if (motorIndex > 3) return;
    if (_emergencyStop) return;

    // Constrain speed
    speed = constrain(speed, -255, 255);

    // Apply inversion if needed
    if (_motors[motorIndex].inverted) {
        speed = -speed;
    }

    // Store current speed
    _motors[motorIndex].currentSpeed = speed;

    // Apply to motor
    applyMotorOutput(motorIndex, speed);
}

void MotorDriver::setAllMotorSpeeds(int16_t speedFL, int16_t speedFR,
                                     int16_t speedRL, int16_t speedRR) {
    setMotorSpeed(MOTOR_FL, speedFL);
    setMotorSpeed(MOTOR_FR, speedFR);
    setMotorSpeed(MOTOR_RL, speedRL);
    setMotorSpeed(MOTOR_RR, speedRR);
}

void MotorDriver::stopMotor(uint8_t motorIndex) {
    if (motorIndex > 3) return;

    _motors[motorIndex].currentSpeed = 0;

    // Coast stop: both inputs LOW
    _pwm->setPWM(_motors[motorIndex].in1_channel, 0, 0);
    _pwm->setPWM(_motors[motorIndex].in2_channel, 0, 0);
}

void MotorDriver::stopAll() {
    for (uint8_t i = 0; i < 4; i++) {
        stopMotor(i);
    }
}

void MotorDriver::emergencyStop() {
    _emergencyStop = true;
    brakeAll();  // Active brake for emergency
}

void MotorDriver::brakeMotor(uint8_t motorIndex) {
    if (motorIndex > 3) return;

    _motors[motorIndex].currentSpeed = 0;

    // Active brake: both inputs HIGH
    _pwm->setPWM(_motors[motorIndex].in1_channel, 0, 4095);
    _pwm->setPWM(_motors[motorIndex].in2_channel, 0, 4095);
}

void MotorDriver::brakeAll() {
    for (uint8_t i = 0; i < 4; i++) {
        brakeMotor(i);
    }
}

int16_t MotorDriver::getMotorSpeed(uint8_t motorIndex) {
    if (motorIndex > 3) return 0;

    int16_t speed = _motors[motorIndex].currentSpeed;

    // Undo inversion for external reporting
    if (_motors[motorIndex].inverted) {
        speed = -speed;
    }

    return speed;
}

bool MotorDriver::isEmergencyStopped() {
    return _emergencyStop;
}

void MotorDriver::applyMotorOutput(uint8_t motorIndex, int16_t speed) {
    uint8_t in1 = _motors[motorIndex].in1_channel;
    uint8_t in2 = _motors[motorIndex].in2_channel;

    if (speed == 0) {
        // Coast: both LOW
        _pwm->setPWM(in1, 0, 0);
        _pwm->setPWM(in2, 0, 0);
    }
    else if (speed > 0) {
        // Apply calibration: scale then offset
        int16_t calibrated = ((int32_t)speed * _motors[motorIndex].pwmScale) / 100;
        calibrated += _motors[motorIndex].pwmOffset;
        calibrated = constrain(calibrated, 0, 255);

        // Forward: IN1 = PWM, IN2 = LOW (fast decay)
        uint16_t pwmValue = speedToPWM(calibrated);
        _pwm->setPWM(in1, 0, pwmValue);
        _pwm->setPWM(in2, 0, 0);
    }
    else {
        // Apply calibration: scale then offset
        int16_t calibrated = ((int32_t)(-speed) * _motors[motorIndex].pwmScale) / 100;
        calibrated += _motors[motorIndex].pwmOffset;
        calibrated = constrain(calibrated, 0, 255);

        // Reverse: IN1 = LOW, IN2 = PWM (fast decay)
        uint16_t pwmValue = speedToPWM(calibrated);
        _pwm->setPWM(in1, 0, 0);
        _pwm->setPWM(in2, 0, pwmValue);
    }
}

uint16_t MotorDriver::speedToPWM(int16_t speed) {
    // Convert 0-255 to 0-4095 (12-bit PWM)
    // Apply a minimum threshold to overcome motor deadzone
    if (speed < 10) return 0;

    // Map 10-255 to ~160-4095 (leave some margin)
    uint32_t pwm = map(speed, 0, 255, 0, 4095);
    return (uint16_t)pwm;
}

#endif // MOTOR_DRIVER_H
