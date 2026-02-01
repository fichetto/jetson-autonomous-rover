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
// Mapping da schema Moebius ARDUINO_HR8833.pdf
// M1=JP1 (canali 0,1), M2=JP2 (canali 2,3), M3=JP3 (canali 4,5), M4=JP4 (canali 6,7)

// Motor M1/FL (Front Left) - JP1 - canali 0,1
#define PWM_FL_IN1  0   // PCA9685 channel 0 (LED0)
#define PWM_FL_IN2  1   // PCA9685 channel 1 (LED1)

// Motor M2/FR (Front Right) - JP2 - canali 2,3
#define PWM_FR_IN1  2   // PCA9685 channel 2 (LED2)
#define PWM_FR_IN2  3   // PCA9685 channel 3 (LED3)

// Motor M3/RL (Rear Left) - JP3 - canali 4,5
#define PWM_RL_IN1  4   // PCA9685 channel 4 (LED4)
#define PWM_RL_IN2  5   // PCA9685 channel 5 (LED5)

// Motor M4/RR (Rear Right) - JP4 - canali 6,7
#define PWM_RR_IN1  6   // PCA9685 channel 6 (LED6)
#define PWM_RR_IN2  7   // PCA9685 channel 7 (LED7)

// Motor inversion flags (set to true if motor runs backwards)
#define INVERT_FL   false
#define INVERT_FR   true    // Right side typically inverted
#define INVERT_RL   false
#define INVERT_RR   true    // Right side typically inverted

// ============================================================================
// Motor PWM Calibration (Open-Loop Speed Matching)
// ============================================================================
// Offset values to compensate for motor differences (-50 to +50)
// Positive = motor runs faster, Negative = motor runs slower
// Calibrate by observing wheel speeds at same PWM command
#define PWM_OFFSET_FL   0
#define PWM_OFFSET_FR   0
#define PWM_OFFSET_RL   0
#define PWM_OFFSET_RR   0

// Scale factors (percentage, 100 = normal)
// Use if one motor is consistently faster/slower at all speeds
#define PWM_SCALE_FL    100
#define PWM_SCALE_FR    100
#define PWM_SCALE_RL    100
#define PWM_SCALE_RR    100

// ============================================================================
// Encoder Configuration
// ============================================================================
// NOTA: La shield Moebius UNO supporta solo 2 encoder (JP1=M1, JP2=M2)
// JP3 e JP4 hanno pin encoder NC (non collegati)!

// Encoder pins (da schema ARDUINO_HR8833.pdf)
// Solo M1 e M2 hanno encoder collegati su Arduino UNO
#define ENCODER_M1_PIN  2   // D2 - ENCODER1 (interrupt INT0)
#define ENCODER_M2_PIN  3   // D3 - ENCODER2 (interrupt INT1)
#define ENCODER_M3_PIN  4   // D4 - NON COLLEGATO su shield UNO!
#define ENCODER_M4_PIN  5   // D5 - NON COLLEGATO su shield UNO!

// Encoder specifications (JGB 520)
#define ENCODER_CPR         11      // Counts per motor revolution
#define MOTOR_GEAR_RATIO    30      // Gear reduction ratio
#define TICKS_PER_REV       (ENCODER_CPR * MOTOR_GEAR_RATIO)  // 330 ticks/wheel rev

// ============================================================================
// PID Configuration (Closed-Loop Velocity Control)
// ============================================================================

// PID Gains per motor (scaled x100 for integer math)
// Tune these values empirically for your specific motors

// Front Left Motor
#define PID_KP_FL   200     // Proportional gain (2.0)
#define PID_KI_FL    50     // Integral gain (0.5)
#define PID_KD_FL    10     // Derivative gain (0.1)

// Front Right Motor
#define PID_KP_FR   200
#define PID_KI_FR    50
#define PID_KD_FR    10

// Rear Left Motor
#define PID_KP_RL   200
#define PID_KI_RL    50
#define PID_KD_RL    10

// Rear Right Motor
#define PID_KP_RR   200
#define PID_KI_RR    50
#define PID_KD_RR    10

// PID Limits
#define PID_INTEGRAL_MAX    10000   // Anti-windup limit
#define PID_RPM_MAX         200     // Maximum target RPM
#define PID_OUTPUT_MAX      255     // Maximum PWM output

// ============================================================================
// Battery Configuration
// ============================================================================

// Battery monitoring pin
#define BATTERY_PIN         A0

// Voltage divider ratio (calibrated from actual measurement)
// Calibration: 12.6V real measured with multimeter
#define VOLTAGE_DIVIDER_RATIO   7.15

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

// Closed-Loop Control (PID)
// Target RPM values: 0-400 mapped to -200 to +200 RPM (offset 200)
#define HREG_TARGET_RPM_FL      20  // Holding[20]: Target RPM Front Left
#define HREG_TARGET_RPM_FR      21  // Holding[21]: Target RPM Front Right
#define HREG_TARGET_RPM_RL      22  // Holding[22]: Target RPM Rear Left
#define HREG_TARGET_RPM_RR      23  // Holding[23]: Target RPM Rear Right
#define HREG_PID_ENABLE         24  // Holding[24]: 0=open-loop, 1=closed-loop

// RPM register offset (200 = 0 RPM, 0 = -200 RPM, 400 = +200 RPM)
#define RPM_REGISTER_OFFSET     200

// Number of holding registers needed
#define HOLDING_REG_COUNT       30

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
