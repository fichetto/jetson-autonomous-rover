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

// ModbusRTU library configuration - MUST be before include
#define INPUT_REGISTER_NUM 25
#define HOLDING_REGISTER_NUM 20
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

// Modbus RTU server
ModbusRTUServer modbusServer;

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

    // Initialize Modbus RTU Server
    // Note: This initializes Serial at the specified baud rate
    modbusServer.startModbusServer(MODBUS_SLAVE_ID, MODBUS_BAUDRATE, Serial, true);

    // Initialize registers to default values
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
    // Poll Modbus - returns true if a write occurred
    bool writeOccurred = modbusServer.communicationLoop();

    if (writeOccurred) {
        // Process motor commands when new data arrives
        processMotorCommands();
        lastMotorCommand = currentMillis;
    }

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
    if (modbusServer.getHoldingValue(HREG_EMERGENCY_STOP) != 0) {
        motors.emergencyStop();
    }

    // -------------------------------------------------------------------------
    // Check for encoder reset command
    // -------------------------------------------------------------------------
    if (modbusServer.getHoldingValue(HREG_RESET_ENCODERS) != 0) {
        encoders.resetAll();
        modbusServer.setHoldingValue(HREG_RESET_ENCODERS, 0);  // Auto-clear
    }
}

// ============================================================================
// Register Initialization
// ============================================================================

void initializeRegisters() {
    // Set motor speeds to stop (255 = stopped in our mapping)
    modbusServer.setHoldingValue(HREG_MOTOR_FL_SPEED, MOTOR_STOP_VALUE);
    modbusServer.setHoldingValue(HREG_MOTOR_FR_SPEED, MOTOR_STOP_VALUE);
    modbusServer.setHoldingValue(HREG_MOTOR_RL_SPEED, MOTOR_STOP_VALUE);
    modbusServer.setHoldingValue(HREG_MOTOR_RR_SPEED, MOTOR_STOP_VALUE);

    // System mode: manual
    modbusServer.setHoldingValue(HREG_SYSTEM_MODE, 0);

    // Emergency stop: released
    modbusServer.setHoldingValue(HREG_EMERGENCY_STOP, 0);

    // Reset encoders flag: cleared
    modbusServer.setHoldingValue(HREG_RESET_ENCODERS, 0);

    // Initialize input registers to 0
    for (int i = 0; i < INPUT_REG_COUNT; i++) {
        modbusServer.setInputValue(i, 0);
    }
}

// ============================================================================
// Motor Command Processing
// ============================================================================

void processMotorCommands() {
    // Check if emergency stop is active
    if (modbusServer.getHoldingValue(HREG_EMERGENCY_STOP) != 0) {
        return;  // Don't process commands during emergency stop
    }

    // Get motor speed register values
    uint16_t regFL = modbusServer.getHoldingValue(HREG_MOTOR_FL_SPEED);
    uint16_t regFR = modbusServer.getHoldingValue(HREG_MOTOR_FR_SPEED);
    uint16_t regRL = modbusServer.getHoldingValue(HREG_MOTOR_RL_SPEED);
    uint16_t regRR = modbusServer.getHoldingValue(HREG_MOTOR_RR_SPEED);

    // Convert register values to signed speeds (-255 to +255)
    // Register mapping: 0-254 = reverse, 255 = stop, 256-510 = forward
    int speedFL = registerToSpeed(regFL);
    int speedFR = registerToSpeed(regFR);
    int speedRL = registerToSpeed(regRL);
    int speedRR = registerToSpeed(regRR);

    // Apply motor speeds
    motors.setMotorSpeed(MOTOR_FL, speedFL);
    motors.setMotorSpeed(MOTOR_FR, speedFR);
    motors.setMotorSpeed(MOTOR_RL, speedRL);
    motors.setMotorSpeed(MOTOR_RR, speedRR);
}

// ============================================================================
// Encoder Register Updates
// ============================================================================

void updateEncoderRegisters() {
    // Update encoder counts and speeds
    encoders.update();

    // Get encoder counts (convert to uint16_t, handle overflow)
    modbusServer.setInputValue(IREG_ENCODER_M1_COUNT, (uint16_t)(encoders.getCount(0) & 0xFFFF));
    modbusServer.setInputValue(IREG_ENCODER_M2_COUNT, (uint16_t)(encoders.getCount(1) & 0xFFFF));
    modbusServer.setInputValue(IREG_ENCODER_M3_COUNT, (uint16_t)(encoders.getCount(2) & 0xFFFF));
    modbusServer.setInputValue(IREG_ENCODER_M4_COUNT, (uint16_t)(encoders.getCount(3) & 0xFFFF));

    // Get encoder speeds (RPM) - convert signed to unsigned with offset
    modbusServer.setInputValue(IREG_ENCODER_M1_SPEED, rpmToRegister(encoders.getRPM(0)));
    modbusServer.setInputValue(IREG_ENCODER_M2_SPEED, rpmToRegister(encoders.getRPM(1)));
    modbusServer.setInputValue(IREG_ENCODER_M3_SPEED, rpmToRegister(encoders.getRPM(2)));
    modbusServer.setInputValue(IREG_ENCODER_M4_SPEED, rpmToRegister(encoders.getRPM(3)));
}

// ============================================================================
// Battery Register Update
// ============================================================================

void updateBatteryRegister() {
    // Read battery voltage and store in mV
    uint16_t voltage_mV = battery.readVoltage_mV();
    modbusServer.setInputValue(IREG_BATTERY_VOLTAGE, voltage_mV);

    // Check for critical battery level
    if (voltage_mV < BATTERY_CRITICAL_MV && voltage_mV > 1000) {
        // Critical battery - trigger emergency stop
        modbusServer.setHoldingValue(HREG_EMERGENCY_STOP, 1);
        motors.emergencyStop();
    }
}

// ============================================================================
// Safety Watchdog
// ============================================================================

void checkWatchdog(unsigned long currentMillis) {
    // If no motor commands received within timeout, stop motors
    if (currentMillis - lastMotorCommand >= WATCHDOG_TIMEOUT) {
        motors.stopAll();

        // Set motor registers to stop
        modbusServer.setHoldingValue(HREG_MOTOR_FL_SPEED, MOTOR_STOP_VALUE);
        modbusServer.setHoldingValue(HREG_MOTOR_FR_SPEED, MOTOR_STOP_VALUE);
        modbusServer.setHoldingValue(HREG_MOTOR_RL_SPEED, MOTOR_STOP_VALUE);
        modbusServer.setHoldingValue(HREG_MOTOR_RR_SPEED, MOTOR_STOP_VALUE);
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
