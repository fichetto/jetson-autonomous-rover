/*
 * Test Modbus Motor - Minimal firmware for debugging
 *
 * Just PCA9685 + Modbus, nothing else
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <ModbusRTU.h>

#define PCA9685_ADDRESS 0x40
#define PWM_FREQUENCY   1000
#define MODBUS_SLAVE_ID 1
#define MODBUS_BAUDRATE 115200

// Objects
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDRESS);
ModbusRTU mb;

// Local storage for motor registers (like working datalogger)
uint16_t motorRegs[4] = {255, 255, 255, 255};  // 255 = stopped

// Debug LED
#define LED_PIN 13
volatile bool callbackCalled = false;

// Write callback - acts immediately
uint16_t onMotorWrite(TRegister* reg, uint16_t val) {
    uint16_t addr = reg->address.address;

    // Toggle LED to show callback was called
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    callbackCalled = true;

    if (addr <= 3) {
        motorRegs[addr] = val;

        // Convert to speed
        int16_t speed = (int16_t)val - 255;

        // Motor inversion for right side
        if (addr == 1 || addr == 3) {
            speed = -speed;
        }

        // Calculate channels
        uint8_t ch1 = addr * 2;
        uint8_t ch2 = addr * 2 + 1;

        // Apply PWM
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
    }

    return val;
}

// Read callback - return from local storage
uint16_t onMotorRead(TRegister* reg, uint16_t val) {
    uint16_t addr = reg->address.address;
    if (addr <= 3) {
        return motorRegs[addr];
    }
    return val;
}

void setup() {
    // Debug LED
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // I2C & PCA9685
    Wire.begin();
    pwm.begin();
    pwm.setPWMFreq(PWM_FREQUENCY);

    // Zero all channels
    for (int i = 0; i < 16; i++) {
        pwm.setPWM(i, 0, 0);
    }

    // Modbus
    Serial.begin(MODBUS_BAUDRATE);
    mb.begin(&Serial);
    mb.slave(MODBUS_SLAVE_ID);

    // Add registers 0-3 for motors
    for (int i = 0; i < 4; i++) {
        mb.addHreg(i, 255);  // Init to stopped
    }

    // Register callbacks
    mb.onSetHreg(0, onMotorWrite, 4);
    mb.onGetHreg(0, onMotorRead, 4);

    // LED on to show ready
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_PIN, LOW);
}

void loop() {
    mb.task();
    yield();
}
