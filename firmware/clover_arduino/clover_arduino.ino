/*
 * ================================================================================
 *                              PROGETTO CLOVER
 *                     Firmware Arduino Uno - Motor Controller
 *                                  v1.1
 * ================================================================================
 *
 * Hardware:
 *   - Arduino Uno (ATmega328P @ 16MHz)
 *   - Moebius 4CH Motor Driver Shield
 *   - PCA9685 PWM Controller (I2C @ 0x40)
 *   - HR8833 Dual H-Bridge (x2)
 *   - JGB 520 Motors with Hall Encoders (x4)
 *   - LiPo Battery 3S2P (11.1V nominal)
 *
 * Communication:
 *   - Modbus RTU Slave (ID: 1, 115200 baud)
 *   - USB Serial connection to Jetson Orin Nano
 *   - Library: modbus-esp8266 (works on Arduino Uno)
 *
 * Pin Mapping:
 *   D0 (RX)  - Modbus RTU Serial RX
 *   D1 (TX)  - Modbus RTU Serial TX
 *   D2       - Encoder M1 (INT0)
 *   D3       - Encoder M2 (INT1)
 *   D4       - Encoder M3 (polling)
 *   D5       - Encoder M4 (polling)
 *   A0       - Battery Voltage (via divider)
 *   A4 (SDA) - I2C Data (PCA9685)
 *   A5 (SCL) - I2C Clock (PCA9685)
 *
 * ================================================================================
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <ModbusRTU.h>

#include "config.h"
#include "motor_driver.h"
#include "encoder.h"
#include "battery.h"
#include "pid_controller.h"

// ============================================================================
// I2C Bus Recovery
// ============================================================================
// Recovers I2C bus when SDA is stuck low due to motor EMI
// Based on: https://www.forward.com.au/pfod/ArduinoProgramming/I2C_ClearBus/

#define SDA_PIN A4
#define SCL_PIN A5

bool clearI2CBus() {
    pinMode(SDA_PIN, INPUT_PULLUP);
    pinMode(SCL_PIN, INPUT_PULLUP);
    delay(2);

    // Check if SCL is held low - loss of arbitration
    if (digitalRead(SCL_PIN) == LOW) {
        return false;  // SCL stuck, need hardware reset
    }

    // Check if SDA is held low
    bool sdaLow = (digitalRead(SDA_PIN) == LOW);
    int clockCount = 20;  // Max clocks to try

    while (sdaLow && (clockCount > 0)) {
        clockCount--;
        // Clock SCL to release SDA
        pinMode(SCL_PIN, INPUT);
        pinMode(SCL_PIN, OUTPUT);
        digitalWrite(SCL_PIN, LOW);
        delayMicroseconds(10);
        pinMode(SCL_PIN, INPUT);
        digitalWrite(SCL_PIN, HIGH);
        delayMicroseconds(10);
        sdaLow = (digitalRead(SDA_PIN) == LOW);
    }

    if (sdaLow) {
        return false;  // SDA still stuck
    }

    // Generate STOP condition
    pinMode(SDA_PIN, INPUT);
    pinMode(SDA_PIN, OUTPUT);
    digitalWrite(SDA_PIN, LOW);
    delayMicroseconds(10);
    pinMode(SCL_PIN, INPUT);
    digitalWrite(SCL_PIN, HIGH);
    delayMicroseconds(10);
    pinMode(SDA_PIN, INPUT);
    digitalWrite(SDA_PIN, HIGH);
    delayMicroseconds(10);

    return true;
}

// reinitI2C() defined after pwm declaration

// ============================================================================
// Global Objects
// ============================================================================

// PCA9685 PWM driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDRESS);

// Motor driver instance
MotorDriver motors(&pwm);

// Encoder manager
EncoderManager encoders;

// Battery monitor
BatteryMonitor battery(BATTERY_PIN);

// PID controller for closed-loop velocity control
PIDController pidController;

// Modbus RTU slave (using modbus-esp8266 library)
ModbusRTU mb;

// ============================================================================
// I2C Reinit (needs pwm to be declared first)
// ============================================================================

void reinitI2C() {
    Wire.end();
    clearI2CBus();
    Wire.begin();
    pwm.begin();
    pwm.setPWMFreq(PWM_FREQUENCY);
    // Zero all PWM channels after reinit
    for (uint8_t ch = 0; ch < 16; ch++) {
        pwm.setPWM(ch, 0, 0);
    }
}

// ============================================================================
// Timing
// ============================================================================

unsigned long lastEncoderUpdate = 0;
unsigned long lastBatteryUpdate = 0;
unsigned long lastWatchdog = 0;
unsigned long lastMotorCommand = 0;
unsigned long lastI2CCheck = 0;
uint8_t i2cErrorCount = 0;

#define I2C_CHECK_INTERVAL  500   // Check I2C every 500ms
#define I2C_ERROR_THRESHOLD 3     // Reinit after 3 consecutive errors

// ============================================================================
// Modbus Callback for Motor Writes
// ============================================================================

// Local storage for motor speeds (like working datalogger pattern)
uint16_t localMotorRegs[4] = {MOTOR_STOP_VALUE, MOTOR_STOP_VALUE,
                               MOTOR_STOP_VALUE, MOTOR_STOP_VALUE};

// Channel mapping arrays (from config.h)
const uint8_t motorCh1[4] = {PWM_FL_IN1, PWM_FR_IN1, PWM_RL_IN1, PWM_RR_IN1};
const uint8_t motorCh2[4] = {PWM_FL_IN2, PWM_FR_IN2, PWM_RL_IN2, PWM_RR_IN2};
const bool motorInvert[4] = {INVERT_FL, INVERT_FR, INVERT_RL, INVERT_RR};

// Direct motor control in callback (same pattern as working datalogger)
uint16_t onMotorWrite(TRegister* reg, uint16_t val) {
    uint16_t address = reg->address.address;

    if (address <= 3) {
        // Store in local array
        localMotorRegs[address] = val;

        // Convert to speed (-255 to +255)
        int16_t speed = (int16_t)val - 255;

        // Get PWM channels from config
        uint8_t ch1 = motorCh1[address];
        uint8_t ch2 = motorCh2[address];

        // Apply motor direction inversion from config
        if (motorInvert[address]) {
            speed = -speed;
        }

        // Apply PWM directly (like working datalogger does with digitalWrite)
        if (speed == 0) {
            pwm.setPWM(ch1, 0, 0);
            pwm.setPWM(ch2, 0, 0);
        } else if (speed > 0) {
            uint16_t pwmVal = map(speed, 0, 255, 0, 4095);
            pwm.setPWM(ch1, 0, pwmVal);
            pwm.setPWM(ch2, 0, 0);
        } else {
            uint16_t pwmVal = map(-speed, 0, 255, 0, 4095);
            pwm.setPWM(ch1, 0, 0);
            pwm.setPWM(ch2, 0, pwmVal);
        }

        // Reset watchdog
        lastMotorCommand = millis();
    }

    return val;
}

// Callback for reading motor registers (return from local storage)
uint16_t onMotorRead(TRegister* reg, uint16_t val) {
    uint16_t address = reg->address.address;
    if (address <= 3) {
        return localMotorRegs[address];
    }
    return val;
}

// ============================================================================
// Setup
// ============================================================================

void setup() {
    // Clear I2C bus before init (in case of previous lockup)
    clearI2CBus();

    // Initialize I2C
    Wire.begin();

    // Initialize PCA9685 with explicit reset
    pwm.begin();

    // CRITICAL: Zero all PWM channels BEFORE setting frequency
    // This prevents random motor movement on power-up
    for (uint8_t ch = 0; ch < 16; ch++) {
        pwm.setPWM(ch, 0, 0);
    }
    delay(5);

    pwm.setPWMFreq(PWM_FREQUENCY);
    delay(10);

    // Zero all channels again after frequency change
    for (uint8_t ch = 0; ch < 16; ch++) {
        pwm.setPWM(ch, 0, 0);
    }
    delay(5);

    // Initialize motors (all stopped)
    motors.begin();
    motors.stopAll();

    // Initialize encoders
    encoders.begin();

    // Initialize PID controller
    pidController.begin();

    // Initialize battery monitor
    battery.begin();

    // Initialize Modbus RTU Slave
    Serial.begin(MODBUS_BAUDRATE);
    mb.begin(&Serial);
    mb.slave(MODBUS_SLAVE_ID);

    // Initialize Modbus registers
    initializeRegisters();

    // Set up Modbus callbacks for motor registers (like working datalogger)
    mb.onSetHreg(0, onMotorWrite, 4);  // Write callback for registers 0-3
    mb.onGetHreg(0, onMotorRead, 4);   // Read callback for registers 0-3

    // Initial watchdog timestamp
    lastWatchdog = millis();
    lastMotorCommand = millis();
}

// ============================================================================
// Main Loop
// ============================================================================

void loop() {
    unsigned long currentMillis = millis();

    // -------------------------------------------------------------------------
    // Modbus Communication
    // -------------------------------------------------------------------------
    mb.task();

    // Motor commands are now handled directly in onMotorWrite callback
    // (same pattern as working datalogger on Raspberry Pi)

    // -------------------------------------------------------------------------
    // Update Encoders and PID Control
    // -------------------------------------------------------------------------
    if (currentMillis - lastEncoderUpdate >= ENCODER_UPDATE_INTERVAL) {
        lastEncoderUpdate = currentMillis;
        updateEncoderRegisters();

        // Run PID control if enabled
        if (mb.Hreg(HREG_PID_ENABLE) != 0) {
            updatePIDControl();
        }
    }

    // -------------------------------------------------------------------------
    // Update Battery Voltage
    // -------------------------------------------------------------------------
    if (currentMillis - lastBatteryUpdate >= BATTERY_UPDATE_INTERVAL) {
        lastBatteryUpdate = currentMillis;
        updateBatteryRegister();
    }

    // -------------------------------------------------------------------------
    // Safety Watchdog - Fixed race condition bug
    // -------------------------------------------------------------------------
    checkWatchdog();

    // -------------------------------------------------------------------------
    // Emergency Stop Check
    // -------------------------------------------------------------------------
    if (mb.Hreg(HREG_EMERGENCY_STOP) != 0) {
        motors.emergencyStop();
    }

    // -------------------------------------------------------------------------
    // Check for encoder reset command
    // -------------------------------------------------------------------------
    if (mb.Hreg(HREG_RESET_ENCODERS) != 0) {
        encoders.resetAll();
        pidController.resetAll();
        mb.Hreg(HREG_RESET_ENCODERS, 0);
    }

    // -------------------------------------------------------------------------
    // I2C Health Check - Recover from EMI-induced bus lockup
    // Only reinit if motors are stopped to avoid interrupting movement
    // -------------------------------------------------------------------------
    if (currentMillis - lastI2CCheck >= I2C_CHECK_INTERVAL) {
        lastI2CCheck = currentMillis;

        // Try to communicate with PCA9685
        Wire.beginTransmission(PCA9685_ADDRESS);
        uint8_t error = Wire.endTransmission();

        if (error != 0) {
            i2cErrorCount++;
            if (i2cErrorCount >= I2C_ERROR_THRESHOLD) {
                // I2C appears stuck, attempt recovery
                reinitI2C();
                // Don't call motors.stopAll() here - let the callback handle motor state
                i2cErrorCount = 0;
            }
        } else {
            i2cErrorCount = 0;  // Reset on success
        }
    }

    yield();
}

// ============================================================================
// Register Initialization
// ============================================================================

void initializeRegisters() {
    // Add Holding Registers (Read/Write)
    mb.addHreg(HREG_MOTOR_FL_SPEED, MOTOR_STOP_VALUE);
    mb.addHreg(HREG_MOTOR_FR_SPEED, MOTOR_STOP_VALUE);
    mb.addHreg(HREG_MOTOR_RL_SPEED, MOTOR_STOP_VALUE);
    mb.addHreg(HREG_MOTOR_RR_SPEED, MOTOR_STOP_VALUE);
    mb.addHreg(HREG_EMERGENCY_STOP, 0);
    mb.addHreg(HREG_SYSTEM_MODE, 0);
    mb.addHreg(HREG_RESET_ENCODERS, 0);

    // PID Control Registers (closed-loop velocity control)
    // Target RPM: value 200 = 0 RPM, 0 = -200 RPM, 400 = +200 RPM
    mb.addHreg(HREG_TARGET_RPM_FL, RPM_REGISTER_OFFSET);  // 0 RPM
    mb.addHreg(HREG_TARGET_RPM_FR, RPM_REGISTER_OFFSET);
    mb.addHreg(HREG_TARGET_RPM_RL, RPM_REGISTER_OFFSET);
    mb.addHreg(HREG_TARGET_RPM_RR, RPM_REGISTER_OFFSET);
    mb.addHreg(HREG_PID_ENABLE, 0);  // Disabled by default (open-loop mode)

    // Add Input Registers (Read Only)
    mb.addIreg(IREG_ENCODER_M1_COUNT, 0);
    mb.addIreg(IREG_ENCODER_M2_COUNT, 0);
    mb.addIreg(IREG_ENCODER_M3_COUNT, 0);
    mb.addIreg(IREG_ENCODER_M4_COUNT, 0);
    mb.addIreg(IREG_ENCODER_M1_SPEED, 32768);  // 0 RPM with offset
    mb.addIreg(IREG_ENCODER_M2_SPEED, 32768);
    mb.addIreg(IREG_ENCODER_M3_SPEED, 32768);
    mb.addIreg(IREG_ENCODER_M4_SPEED, 32768);
    mb.addIreg(IREG_BATTERY_VOLTAGE, 0);
}

// ============================================================================
// Motor Command Processing
// ============================================================================

// Previous motor values for change detection
static uint16_t prevFL = MOTOR_STOP_VALUE;
static uint16_t prevFR = MOTOR_STOP_VALUE;
static uint16_t prevRL = MOTOR_STOP_VALUE;
static uint16_t prevRR = MOTOR_STOP_VALUE;

void checkMotorCommands() {
    // Check if emergency stop is active
    if (mb.Hreg(HREG_EMERGENCY_STOP) != 0) {
        return;  // Don't process commands during emergency stop
    }

    // Skip direct motor commands when PID closed-loop is enabled
    if (mb.Hreg(HREG_PID_ENABLE) != 0) {
        return;  // PID controller handles motor speeds
    }

    // Get current motor speed register values
    uint16_t regFL = mb.Hreg(HREG_MOTOR_FL_SPEED);
    uint16_t regFR = mb.Hreg(HREG_MOTOR_FR_SPEED);
    uint16_t regRL = mb.Hreg(HREG_MOTOR_RL_SPEED);
    uint16_t regRR = mb.Hreg(HREG_MOTOR_RR_SPEED);

    // Check if any value changed
    if (regFL != prevFL || regFR != prevFR || regRL != prevRL || regRR != prevRR) {
        // Convert register values to signed speeds (-255 to +255)
        int speedFL = registerToSpeed(regFL);
        int speedFR = registerToSpeed(regFR);
        int speedRL = registerToSpeed(regRL);
        int speedRR = registerToSpeed(regRR);

        // Apply motor speeds
        motors.setMotorSpeed(MOTOR_FL, speedFL);
        motors.setMotorSpeed(MOTOR_FR, speedFR);
        motors.setMotorSpeed(MOTOR_RL, speedRL);
        motors.setMotorSpeed(MOTOR_RR, speedRR);

        // Update previous values
        prevFL = regFL;
        prevFR = regFR;
        prevRL = regRL;
        prevRR = regRR;

        // Reset watchdog
        lastMotorCommand = millis();
    }
}

// ============================================================================
// Encoder Register Updates
// ============================================================================

void updateEncoderRegisters() {
    // Update encoder counts and speeds
    encoders.update();

    // Get encoder counts (convert to uint16_t, handle overflow)
    mb.Ireg(IREG_ENCODER_M1_COUNT, (uint16_t)(encoders.getCount(0) & 0xFFFF));
    mb.Ireg(IREG_ENCODER_M2_COUNT, (uint16_t)(encoders.getCount(1) & 0xFFFF));
    mb.Ireg(IREG_ENCODER_M3_COUNT, (uint16_t)(encoders.getCount(2) & 0xFFFF));
    mb.Ireg(IREG_ENCODER_M4_COUNT, (uint16_t)(encoders.getCount(3) & 0xFFFF));

    // Get encoder speeds (RPM) - convert signed to unsigned with offset
    mb.Ireg(IREG_ENCODER_M1_SPEED, rpmToRegister(encoders.getRPM(0)));
    mb.Ireg(IREG_ENCODER_M2_SPEED, rpmToRegister(encoders.getRPM(1)));
    mb.Ireg(IREG_ENCODER_M3_SPEED, rpmToRegister(encoders.getRPM(2)));
    mb.Ireg(IREG_ENCODER_M4_SPEED, rpmToRegister(encoders.getRPM(3)));
}

// ============================================================================
// PID Closed-Loop Control
// ============================================================================

void updatePIDControl() {
    // Check if emergency stop is active
    if (mb.Hreg(HREG_EMERGENCY_STOP) != 0) {
        pidController.resetAll();
        return;
    }

    // Read target RPM from Modbus registers and convert from offset encoding
    int16_t targetFL = (int16_t)mb.Hreg(HREG_TARGET_RPM_FL) - RPM_REGISTER_OFFSET;
    int16_t targetFR = (int16_t)mb.Hreg(HREG_TARGET_RPM_FR) - RPM_REGISTER_OFFSET;
    int16_t targetRL = (int16_t)mb.Hreg(HREG_TARGET_RPM_RL) - RPM_REGISTER_OFFSET;
    int16_t targetRR = (int16_t)mb.Hreg(HREG_TARGET_RPM_RR) - RPM_REGISTER_OFFSET;

    // Set PID targets
    pidController.setTarget(MOTOR_FL, targetFL);
    pidController.setTarget(MOTOR_FR, targetFR);
    pidController.setTarget(MOTOR_RL, targetRL);
    pidController.setTarget(MOTOR_RR, targetRR);

    // Get current measured RPM from encoders
    int16_t measuredFL = encoders.getRPM(MOTOR_FL);
    int16_t measuredFR = encoders.getRPM(MOTOR_FR);
    int16_t measuredRL = encoders.getRPM(MOTOR_RL);
    int16_t measuredRR = encoders.getRPM(MOTOR_RR);

    // Compute PID outputs
    int16_t outputFL = pidController.compute(MOTOR_FL, measuredFL);
    int16_t outputFR = pidController.compute(MOTOR_FR, measuredFR);
    int16_t outputRL = pidController.compute(MOTOR_RL, measuredRL);
    int16_t outputRR = pidController.compute(MOTOR_RR, measuredRR);

    // Apply motor speeds (PID output is already in -255 to +255 range)
    motors.setMotorSpeed(MOTOR_FL, outputFL);
    motors.setMotorSpeed(MOTOR_FR, outputFR);
    motors.setMotorSpeed(MOTOR_RL, outputRL);
    motors.setMotorSpeed(MOTOR_RR, outputRR);

    // Reset watchdog when PID is active
    lastMotorCommand = millis();
}

// ============================================================================
// Battery Register Update
// ============================================================================

void updateBatteryRegister() {
    // Read battery voltage and store in mV
    uint16_t voltage_mV = battery.readVoltage_mV();
    mb.Ireg(IREG_BATTERY_VOLTAGE, voltage_mV);

    // NOTE: Battery critical check temporarily disabled for testing
    // TODO: Re-enable once voltage divider is properly calibrated
    // Check for critical battery level
    // if (voltage_mV < BATTERY_CRITICAL_MV && voltage_mV > 1000) {
    //     // Critical battery - trigger emergency stop
    //     mb.Hreg(HREG_EMERGENCY_STOP, 1);
    //     motors.emergencyStop();
    // }
}

// ============================================================================
// Safety Watchdog
// ============================================================================

void checkWatchdog() {
    // Use fresh millis() to avoid race condition with callback
    // The callback updates lastMotorCommand during mb.task(), which runs
    // AFTER currentMillis was captured at loop start. Using stale currentMillis
    // causes unsigned underflow: (old_time - new_time) = huge number = trigger!
    unsigned long now = millis();

    // Only trigger if lastMotorCommand is in the past (not just updated by callback)
    if (now - lastMotorCommand >= WATCHDOG_TIMEOUT) {
        // Stop motors using direct PWM (same as callback does)
        for (uint8_t i = 0; i < 4; i++) {
            uint8_t ch1 = motorCh1[i];
            uint8_t ch2 = motorCh2[i];
            pwm.setPWM(ch1, 0, 0);
            pwm.setPWM(ch2, 0, 0);
            localMotorRegs[i] = MOTOR_STOP_VALUE;
        }

        // Reset watchdog timestamp
        lastMotorCommand = now;
    }
}

// ============================================================================
// Utility Functions
// ============================================================================

// Convert register value (0-510) to signed speed (-255 to +255)
int registerToSpeed(uint16_t regValue) {
    if (regValue > 510) regValue = 510;
    return (int)regValue - 255;
}

// Convert signed speed to register value
uint16_t speedToRegister(int speed) {
    speed = constrain(speed, -255, 255);
    return (uint16_t)(speed + 255);
}

// Convert signed RPM to register value (using offset 32768)
uint16_t rpmToRegister(int16_t rpm) {
    return (uint16_t)(rpm + 32768);
}

// Convert register value to signed RPM
int16_t registerToRPM(uint16_t regValue) {
    return (int16_t)(regValue - 32768);
}
