/*
 * ================================================================================
 *                              PROGETTO CLOVER
 *                        Configuration Header File
 * ================================================================================
 */

#ifndef CLOVER_CONFIG_H
#define CLOVER_CONFIG_H

// ============================================================================
// Hardware Configuration
// ============================================================================

// PCA9685 PWM Controller
#define PCA9685_ADDRESS     0x40    // Default I2C address
#define PWM_FREQUENCY       1000    // Hz (1 kHz for DC motors)
#define PWM_RESOLUTION      4096    // 12-bit resolution

// ============================================================================
// Motor Configuration - PCA9685 Channel Mapping
// ============================================================================
// Each motor uses 2 PWM channels on PCA9685 for HR8833 control
// HR8833 #1: Motors FL and FR
// HR8833 #2: Motors RL and RR

// Motor indices
#define MOTOR_FL    0   // Front Left (M1)
#define MOTOR_FR    1   // Front Right (M2)
#define MOTOR_RL    2   // Rear Left (M3)
#define MOTOR_RR    3   // Rear Right (M4)

// PCA9685 channel mapping for HR8833 inputs
// NOTA: Mapping corretto dopo test fisico (coppie invertite)
// Motor FL (Front Left) - canali 2,3
#define PWM_FL_IN1  2   // PCA9685 channel 2
#define PWM_FL_IN2  3   // PCA9685 channel 3

// Motor FR (Front Right) - canali 0,1
#define PWM_FR_IN1  0   // PCA9685 channel 0
#define PWM_FR_IN2  1   // PCA9685 channel 1

// Motor RL (Rear Left) - canali 6,7
#define PWM_RL_IN1  6   // PCA9685 channel 6
#define PWM_RL_IN2  7   // PCA9685 channel 7

// Motor RR (Rear Right) - canali 4,5
#define PWM_RR_IN1  4   // PCA9685 channel 4
#define PWM_RR_IN2  5   // PCA9685 channel 5

// Motor inversion flags (set to true if motor runs backwards)
#define INVERT_FL   false
#define INVERT_FR   true    // Right side typically inverted
#define INVERT_RL   false
#define INVERT_RR   true    // Right side typically inverted

// ============================================================================
// Encoder Configuration
// ============================================================================

// Encoder pins
#define ENCODER_M1_PIN  2   // D2 - INT0 (interrupt)
#define ENCODER_M2_PIN  3   // D3 - INT1 (interrupt)
#define ENCODER_M3_PIN  4   // D4 - polling
#define ENCODER_M4_PIN  5   // D5 - polling

// Encoder specifications (JGB 520)
#define ENCODER_CPR         11      // Counts per motor revolution
#define MOTOR_GEAR_RATIO    30      // Gear reduction ratio
#define TICKS_PER_REV       (ENCODER_CPR * MOTOR_GEAR_RATIO)  // 330 ticks/wheel rev

// ============================================================================
// Battery Configuration
// ============================================================================

// Battery monitoring pin
#define BATTERY_PIN         A0

// Voltage divider ratio (calibrated from actual measurement)
// Calibration: 9.99V real → adjusted to read correctly
#define VOLTAGE_DIVIDER_RATIO   4.86

// LiPo 3S2P Battery thresholds (in mV)
#define BATTERY_MAX_MV      12600   // Fully charged
#define BATTERY_NOMINAL_MV  11100   // Nominal
#define BATTERY_LOW_MV      10000   // Low warning
#define BATTERY_CRITICAL_MV 9000    // Critical - shutdown

// ADC reference voltage (mV)
#define ADC_REF_MV          5000    // 5V for Arduino Uno

// ============================================================================
// Modbus Configuration
// ============================================================================

#define MODBUS_SLAVE_ID     1
#define MODBUS_BAUDRATE     115200

// Motor speed stop value (255 = stopped in 0-510 mapping)
#define MOTOR_STOP_VALUE    255

// ============================================================================
// Modbus Register Map (ModbusRTU library format)
// ============================================================================
// This library uses separate Input and Holding register arrays starting from 0
// We need to map our logical addresses to array indices

// --- HOLDING REGISTERS (Read/Write) ---
// Motor Speed Commands: Values 0-510 where 255=stop
#define HREG_MOTOR_FL_SPEED     0   // Holding[0]
#define HREG_MOTOR_FR_SPEED     1   // Holding[1]
#define HREG_MOTOR_RL_SPEED     2   // Holding[2]
#define HREG_MOTOR_RR_SPEED     3   // Holding[3]

// System Control
#define HREG_EMERGENCY_STOP     10  // Holding[10]: 0=normal, 1=stop
#define HREG_SYSTEM_MODE        11  // Holding[11]: 0=manual, 1=auto
#define HREG_RESET_ENCODERS     12  // Holding[12]: Write 1 to reset all encoders

// Number of holding registers needed
#define HOLDING_REG_COUNT       20

// --- INPUT REGISTERS (Read Only) ---
// Encoder Counts (16-bit, wraps around)
#define IREG_ENCODER_M1_COUNT   0   // Input[0]
#define IREG_ENCODER_M2_COUNT   1   // Input[1]
#define IREG_ENCODER_M3_COUNT   2   // Input[2]
#define IREG_ENCODER_M4_COUNT   3   // Input[3]

// Encoder Speeds (RPM, signed via offset 32768)
#define IREG_ENCODER_M1_SPEED   10  // Input[10]
#define IREG_ENCODER_M2_SPEED   11  // Input[11]
#define IREG_ENCODER_M3_SPEED   12  // Input[12]
#define IREG_ENCODER_M4_SPEED   13  // Input[13]

// Battery
#define IREG_BATTERY_VOLTAGE    20  // Input[20]: mV

// Number of input registers needed
#define INPUT_REG_COUNT         25

// ============================================================================
// Timing Configuration
// ============================================================================

#define ENCODER_UPDATE_INTERVAL     50      // ms (20 Hz)
#define BATTERY_UPDATE_INTERVAL     1000    // ms (1 Hz)
#define WATCHDOG_TIMEOUT            10000   // ms (10 sec - for testing)

// ============================================================================
// Debug Configuration
// ============================================================================

// Uncomment to enable debug output (disables Modbus on Serial!)
// #define DEBUG_ENABLED

#ifdef DEBUG_ENABLED
    #define DEBUG_PRINT(x)      Serial.print(x)
    #define DEBUG_PRINTLN(x)    Serial.println(x)
#else
    #define DEBUG_PRINT(x)
    #define DEBUG_PRINTLN(x)
#endif

#endif // CLOVER_CONFIG_H
