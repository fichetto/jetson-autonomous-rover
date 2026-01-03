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

// Modbus RTU slave (using modbus-esp8266 library)
ModbusRTU mb;

// ============================================================================
// Timing
// ============================================================================

unsigned long lastEncoderUpdate = 0;
unsigned long lastBatteryUpdate = 0;
unsigned long lastWatchdog = 0;
unsigned long lastMotorCommand = 0;

// ============================================================================
// Setup
// ============================================================================

void setup() {
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

    // Initialize battery monitor
    battery.begin();

    // Initialize Modbus RTU Slave
    Serial.begin(MODBUS_BAUDRATE);
    mb.begin(&Serial);
    mb.slave(MODBUS_SLAVE_ID);

    // Initialize Modbus registers
    initializeRegisters();

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

    // Check if motor commands changed
    checkMotorCommands();

    // -------------------------------------------------------------------------
    // Update Encoders
    // -------------------------------------------------------------------------
    if (currentMillis - lastEncoderUpdate >= ENCODER_UPDATE_INTERVAL) {
        lastEncoderUpdate = currentMillis;
        updateEncoderRegisters();
    }

    // -------------------------------------------------------------------------
    // Update Battery Voltage
    // -------------------------------------------------------------------------
    if (currentMillis - lastBatteryUpdate >= BATTERY_UPDATE_INTERVAL) {
        lastBatteryUpdate = currentMillis;
        updateBatteryRegister();
    }

    // -------------------------------------------------------------------------
    // Safety Watchdog
    // -------------------------------------------------------------------------
    checkWatchdog(currentMillis);

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
        mb.Hreg(HREG_RESET_ENCODERS, 0);  // Auto-clear
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

void checkWatchdog(unsigned long currentMillis) {
    // If no motor commands received within timeout, stop motors
    if (currentMillis - lastMotorCommand >= WATCHDOG_TIMEOUT) {
        motors.stopAll();

        // Set motor registers to stop
        mb.Hreg(HREG_MOTOR_FL_SPEED, MOTOR_STOP_VALUE);
        mb.Hreg(HREG_MOTOR_FR_SPEED, MOTOR_STOP_VALUE);
        mb.Hreg(HREG_MOTOR_RL_SPEED, MOTOR_STOP_VALUE);
        mb.Hreg(HREG_MOTOR_RR_SPEED, MOTOR_STOP_VALUE);

        // Update previous values
        prevFL = prevFR = prevRL = prevRR = MOTOR_STOP_VALUE;

        // Reset watchdog to avoid repeated stops
        lastMotorCommand = currentMillis;
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
