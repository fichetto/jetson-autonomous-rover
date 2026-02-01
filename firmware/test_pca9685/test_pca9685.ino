/*
 * Test PCA9685 - Verifica diretta del controller PWM
 * Moebius Shield con HR8833 motor drivers
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define PCA9685_ADDRESS 0x40

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDRESS);

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    Serial.println();
    Serial.println("=================================");
    Serial.println("    TEST PCA9685 / MOEBIUS");
    Serial.println("=================================");
    Serial.println();

    // Scan I2C bus
    Serial.println("Scanning I2C bus...");
    Wire.begin();

    byte deviceCount = 0;
    for (byte addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        byte error = Wire.endTransmission();
        if (error == 0) {
            Serial.print("  Device found at 0x");
            if (addr < 16) Serial.print("0");
            Serial.println(addr, HEX);
            deviceCount++;
        }
    }

    if (deviceCount == 0) {
        Serial.println("  NESSUN DISPOSITIVO I2C TROVATO!");
        Serial.println("  Controlla i collegamenti della shield Moebius");
        while(1) delay(1000);
    }

    Serial.println();

    // Initialize PCA9685
    Serial.println("Initializing PCA9685...");
    pwm.begin();
    pwm.setPWMFreq(1000);  // 1kHz for DC motors

    // Zero all channels
    for (int i = 0; i < 16; i++) {
        pwm.setPWM(i, 0, 0);
    }

    Serial.println("PCA9685 initialized!");
    Serial.println();
    Serial.println("Premi un tasto per testare i motori:");
    Serial.println("  1 = Motor M1 (FL) - canali 0,1");
    Serial.println("  2 = Motor M2 (FR) - canali 2,3");
    Serial.println("  3 = Motor M3 (RL) - canali 4,5");
    Serial.println("  4 = Motor M4 (RR) - canali 6,7");
    Serial.println("  a = Tutti i motori");
    Serial.println("  s = STOP tutti");
    Serial.println("  r = Test rampa");
    Serial.println();
}

void setMotor(int ch1, int ch2, int speed) {
    // speed: -4095 to +4095
    if (speed == 0) {
        pwm.setPWM(ch1, 0, 0);
        pwm.setPWM(ch2, 0, 0);
    } else if (speed > 0) {
        pwm.setPWM(ch1, 0, speed);
        pwm.setPWM(ch2, 0, 0);
    } else {
        pwm.setPWM(ch1, 0, 0);
        pwm.setPWM(ch2, 0, -speed);
    }
}

void stopAll() {
    for (int i = 0; i < 8; i++) {
        pwm.setPWM(i, 0, 0);
    }
    Serial.println("STOP - tutti i canali a 0");
}

void loop() {
    if (Serial.available()) {
        char cmd = Serial.read();

        switch (cmd) {
            case '1':
                Serial.println("Motor M1 (FL) - canali 0,1 - AVANTI 50%");
                setMotor(0, 1, 2048);
                break;

            case '2':
                Serial.println("Motor M2 (FR) - canali 2,3 - AVANTI 50%");
                setMotor(2, 3, 2048);
                break;

            case '3':
                Serial.println("Motor M3 (RL) - canali 4,5 - AVANTI 50%");
                setMotor(4, 5, 2048);
                break;

            case '4':
                Serial.println("Motor M4 (RR) - canali 6,7 - AVANTI 50%");
                setMotor(6, 7, 2048);
                break;

            case 'a':
            case 'A':
                Serial.println("TUTTI i motori - AVANTI 50%");
                setMotor(0, 1, 2048);
                setMotor(2, 3, 2048);
                setMotor(4, 5, 2048);
                setMotor(6, 7, 2048);
                break;

            case 's':
            case 'S':
                stopAll();
                break;

            case 'r':
            case 'R':
                Serial.println("Test RAMPA - PWM crescente...");
                for (int pwmVal = 0; pwmVal <= 4095; pwmVal += 500) {
                    Serial.print("  PWM = ");
                    Serial.println(pwmVal);
                    setMotor(0, 1, pwmVal);
                    setMotor(2, 3, pwmVal);
                    setMotor(4, 5, pwmVal);
                    setMotor(6, 7, pwmVal);
                    delay(1000);
                }
                stopAll();
                Serial.println("Rampa completata");
                break;

            case 'i':
            case 'I':
                Serial.println("Test INDIETRO - tutti i motori");
                setMotor(0, 1, -2048);
                setMotor(2, 3, -2048);
                setMotor(4, 5, -2048);
                setMotor(6, 7, -2048);
                break;
        }
    }
}
