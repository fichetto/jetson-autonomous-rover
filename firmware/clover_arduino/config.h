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
// Motor FL (Front Left) - HR8833 #1, Channel A
#define PWM_FL_IN1  0   // PCA9685 channel 0
#define PWM_FL_IN2  1   // PCA9685 channel 1

// Motor FR (Front Right) - HR8833 #1, Channel B
#define PWM_FR_IN1  2   // PCA9685 channel 2
#define PWM_FR_IN2  3   // PCA9685 channel 3

// Motor RL (Rear Left) - HR8833 #2, Channel A
#define PWM_RL_IN1  4   // PCA9685 channel 4
#define PWM_RL_IN2  5   // PCA9685 channel 5

// Motor RR (Rear Right) - HR8833 #2, Channel B
#define PWM_RR_IN1  6   // PCA9685 channel 6
#define PWM_RR_IN2  7   // PCA9685 channel 7

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

// Voltage divider ratio (adjust based on actual resistors)
// If using 10K + 3.3K divider: ratio = (10K + 3.3K) / 3.3K = 4.03
#define VOLTAGE_DIVIDER_RATIO   4.03

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

// Register count (must cover all addresses up to highest used)
#define MODBUS_REGISTER_COUNT   500

// Motor speed stop value (255 = stopped in 0-510 mapping)
#define MOTOR_STOP_VALUE    255

// ============================================================================
// Modbus Register Map
// ============================================================================
// Must match Python client (modbus_client.py)

// Motor Speed Commands (Holding Registers - Read/Write)
// Values: 0-510 where 255=stop, 0-254=reverse, 256-510=forward
#define REG_MOTOR_FL_SPEED      100     // M1 Front Left
#define REG_MOTOR_FR_SPEED      101     // M2 Front Right
#define REG_MOTOR_RL_SPEED      102     // M3 Rear Left
#define REG_MOTOR_RR_SPEED      103     // M4 Rear Right

// Encoder Counts (Input Registers - Read Only)
#define REG_ENCODER_M1_COUNT    200
#define REG_ENCODER_M2_COUNT    201
#define REG_ENCODER_M3_COUNT    202
#define REG_ENCODER_M4_COUNT    203

// Encoder Speeds in RPM (Input Registers - Read Only)
#define REG_ENCODER_M1_SPEED    210
#define REG_ENCODER_M2_SPEED    211
#define REG_ENCODER_M3_SPEED    212
#define REG_ENCODER_M4_SPEED    213

// System Status (Holding Registers - Read/Write)
#define REG_EMERGENCY_STOP      300     // 0=normal, 1=stop
#define REG_SYSTEM_MODE         301     // 0=manual, 1=auto
#define REG_BATTERY_VOLTAGE     302     // mV (divide by 1000 for V)

// Ultrasonic Sensors (Input Registers - Reserved for expansion)
#define REG_ULTRASONIC_FL       400
#define REG_ULTRASONIC_FC       401
#define REG_ULTRASONIC_FR       402
#define REG_ULTRASONIC_RL       403
#define REG_ULTRASONIC_RC       404
#define REG_ULTRASONIC_RR       405

// ============================================================================
// Timing Configuration
// ============================================================================

#define ENCODER_UPDATE_INTERVAL     50      // ms (20 Hz)
#define BATTERY_UPDATE_INTERVAL     1000    // ms (1 Hz)
#define WATCHDOG_TIMEOUT            2000    // ms (stop motors if no commands)

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
