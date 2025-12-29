/*
 * ================================================================================
 *                              PROGETTO CLOVER
 *                     Firmware Arduino Uno - Motor Controller
 *                                  v1.0
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
#include <ModbusRtu.h>

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

// Modbus RTU slave
Modbus slave(MODBUS_SLAVE_ID, Serial, 0);  // ID=1, Serial port, no TX enable pin

// Modbus register array
uint16_t modbusRegisters[MODBUS_REGISTER_COUNT];

// ============================================================================
// Timing
// ============================================================================

unsigned long lastEncoderUpdate = 0;
unsigned long lastBatteryUpdate = 0;
unsigned long lastWatchdog = 0;

// ============================================================================
// Setup
// ============================================================================

void setup() {
    // Initialize Serial for Modbus RTU
    Serial.begin(MODBUS_BAUDRATE);

    // Initialize I2C
    Wire.begin();

    // Initialize PCA9685
    pwm.begin();
    pwm.setPWMFreq(PWM_FREQUENCY);
    delay(10);

    // Initialize motors (all stopped)
    motors.begin();
    motors.stopAll();

    // Initialize encoders
    encoders.begin();

    // Initialize battery monitor
    battery.begin();

    // Initialize Modbus
    slave.start();

    // Initialize registers to default values
    initializeRegisters();

    // Initial watchdog timestamp
    lastWatchdog = millis();
}

// ============================================================================
// Main Loop
// ============================================================================

void loop() {
    unsigned long currentMillis = millis();

    // -------------------------------------------------------------------------
    // Modbus Communication
    // -------------------------------------------------------------------------
    // Poll Modbus and handle requests
    slave.poll(modbusRegisters, MODBUS_REGISTER_COUNT);

    // -------------------------------------------------------------------------
    // Process Motor Commands
    // -------------------------------------------------------------------------
    processMotorCommands();

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
    if (modbusRegisters[REG_EMERGENCY_STOP] != 0) {
        motors.emergencyStop();
    }
}

// ============================================================================
// Register Initialization
// ============================================================================

void initializeRegisters() {
    // Clear all registers
    for (int i = 0; i < MODBUS_REGISTER_COUNT; i++) {
        modbusRegisters[i] = 0;
    }

    // Set motor speeds to stop (255 = stopped in our mapping)
    modbusRegisters[REG_MOTOR_FL_SPEED] = MOTOR_STOP_VALUE;
    modbusRegisters[REG_MOTOR_FR_SPEED] = MOTOR_STOP_VALUE;
    modbusRegisters[REG_MOTOR_RL_SPEED] = MOTOR_STOP_VALUE;
    modbusRegisters[REG_MOTOR_RR_SPEED] = MOTOR_STOP_VALUE;

    // System mode: manual
    modbusRegisters[REG_SYSTEM_MODE] = 0;

    // Emergency stop: released
    modbusRegisters[REG_EMERGENCY_STOP] = 0;
}

// ============================================================================
// Motor Command Processing
// ============================================================================

void processMotorCommands() {
    // Check if emergency stop is active
    if (modbusRegisters[REG_EMERGENCY_STOP] != 0) {
        return;  // Don't process commands during emergency stop
    }

    // Convert register values to signed speeds (-255 to +255)
    // Register mapping: 0-254 = reverse, 255 = stop, 256-510 = forward
    int speedFL = registerToSpeed(modbusRegisters[REG_MOTOR_FL_SPEED]);
    int speedFR = registerToSpeed(modbusRegisters[REG_MOTOR_FR_SPEED]);
    int speedRL = registerToSpeed(modbusRegisters[REG_MOTOR_RL_SPEED]);
    int speedRR = registerToSpeed(modbusRegisters[REG_MOTOR_RR_SPEED]);

    // Apply motor speeds
    motors.setMotorSpeed(MOTOR_FL, speedFL);
    motors.setMotorSpeed(MOTOR_FR, speedFR);
    motors.setMotorSpeed(MOTOR_RL, speedRL);
    motors.setMotorSpeed(MOTOR_RR, speedRR);

    // Update watchdog (commands received)
    lastWatchdog = millis();
}

// ============================================================================
// Encoder Register Updates
// ============================================================================

void updateEncoderRegisters() {
    // Update encoder counts
    encoders.update();

    // Get encoder counts (convert to uint16_t, handle overflow)
    modbusRegisters[REG_ENCODER_M1_COUNT] = (uint16_t)(encoders.getCount(0) & 0xFFFF);
    modbusRegisters[REG_ENCODER_M2_COUNT] = (uint16_t)(encoders.getCount(1) & 0xFFFF);
    modbusRegisters[REG_ENCODER_M3_COUNT] = (uint16_t)(encoders.getCount(2) & 0xFFFF);
    modbusRegisters[REG_ENCODER_M4_COUNT] = (uint16_t)(encoders.getCount(3) & 0xFFFF);

    // Get encoder speeds (RPM)
    modbusRegisters[REG_ENCODER_M1_SPEED] = speedToRegister(encoders.getRPM(0));
    modbusRegisters[REG_ENCODER_M2_SPEED] = speedToRegister(encoders.getRPM(1));
    modbusRegisters[REG_ENCODER_M3_SPEED] = speedToRegister(encoders.getRPM(2));
    modbusRegisters[REG_ENCODER_M4_SPEED] = speedToRegister(encoders.getRPM(3));
}

// ============================================================================
// Battery Register Update
// ============================================================================

void updateBatteryRegister() {
    // Read battery voltage and store in mV
    uint16_t voltage_mV = battery.readVoltage_mV();
    modbusRegisters[REG_BATTERY_VOLTAGE] = voltage_mV;

    // Check for critical battery level
    if (voltage_mV < BATTERY_CRITICAL_MV && voltage_mV > 1000) {
        // Critical battery - trigger emergency stop
        modbusRegisters[REG_EMERGENCY_STOP] = 1;
        motors.emergencyStop();
    }
}

// ============================================================================
// Safety Watchdog
// ============================================================================

void checkWatchdog(unsigned long currentMillis) {
    // If no commands received within timeout, stop motors
    if (currentMillis - lastWatchdog >= WATCHDOG_TIMEOUT) {
        motors.stopAll();

        // Set motor registers to stop
        modbusRegisters[REG_MOTOR_FL_SPEED] = MOTOR_STOP_VALUE;
        modbusRegisters[REG_MOTOR_FR_SPEED] = MOTOR_STOP_VALUE;
        modbusRegisters[REG_MOTOR_RL_SPEED] = MOTOR_STOP_VALUE;
        modbusRegisters[REG_MOTOR_RR_SPEED] = MOTOR_STOP_VALUE;
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
