/*
 * Test PCA9685 - Verifica che i motori funzionino
 * Senza Modbus, solo test diretto PWM
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define PCA9685_ADDRESS 0x40
#define PWM_FREQUENCY 1000

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDRESS);

void setup() {
    Serial.begin(115200);
    Serial.println("Test PCA9685 Motor Driver");

    Wire.begin();
    pwm.begin();

    // Zero tutti i canali
    for (uint8_t ch = 0; ch < 16; ch++) {
        pwm.setPWM(ch, 0, 0);
    }
    delay(100);

    pwm.setPWMFreq(PWM_FREQUENCY);
    delay(100);

    Serial.println("PCA9685 inizializzato");
    Serial.println("Comandi: 1=avanti, 2=indietro, 0=stop");
}

void loop() {
    if (Serial.available()) {
        char cmd = Serial.read();

        if (cmd == '1') {
            Serial.println("AVANTI - tutti i motori");
            // Motor FL (canali 0,1) - forward
            pwm.setPWM(0, 0, 2048);  // IN1 = PWM
            pwm.setPWM(1, 0, 0);     // IN2 = LOW
            // Motor FR (canali 2,3) - forward (invertito)
            pwm.setPWM(2, 0, 0);
            pwm.setPWM(3, 0, 2048);
            // Motor RL (canali 4,5) - forward
            pwm.setPWM(4, 0, 2048);
            pwm.setPWM(5, 0, 0);
            // Motor RR (canali 6,7) - forward (invertito)
            pwm.setPWM(6, 0, 0);
            pwm.setPWM(7, 0, 2048);
        }
        else if (cmd == '2') {
            Serial.println("INDIETRO - tutti i motori");
            // Motor FL - reverse
            pwm.setPWM(0, 0, 0);
            pwm.setPWM(1, 0, 2048);
            // Motor FR - reverse (invertito)
            pwm.setPWM(2, 0, 2048);
            pwm.setPWM(3, 0, 0);
            // Motor RL - reverse
            pwm.setPWM(4, 0, 0);
            pwm.setPWM(5, 0, 2048);
            // Motor RR - reverse (invertito)
            pwm.setPWM(6, 0, 2048);
            pwm.setPWM(7, 0, 0);
        }
        else if (cmd == '0') {
            Serial.println("STOP");
            for (uint8_t ch = 0; ch < 8; ch++) {
                pwm.setPWM(ch, 0, 0);
            }
        }
        else if (cmd == 't') {
            // Test singolo motore FL
            Serial.println("Test solo FL avanti 2 sec...");
            pwm.setPWM(0, 0, 2048);
            pwm.setPWM(1, 0, 0);
            delay(2000);
            pwm.setPWM(0, 0, 0);
            pwm.setPWM(1, 0, 0);
            Serial.println("Stop");
        }
    }
}
