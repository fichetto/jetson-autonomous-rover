/*
 * Test sequenziale canali PCA9685
 * Prova ogni canale uno alla volta per trovare quali controllano i motori
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

void setup() {
    Serial.begin(115200);
    Wire.begin();
    pwm.begin();

    // Zero tutti i canali
    for (uint8_t ch = 0; ch < 16; ch++) {
        pwm.setPWM(ch, 0, 0);
    }

    pwm.setPWMFreq(1000);
    delay(100);

    Serial.println("Test canali PCA9685");
    Serial.println("Premi un tasto per testare il prossimo canale");
    Serial.println("Ogni canale verrà attivato per 2 secondi");
}

int currentChannel = 0;

void loop() {
    if (Serial.available()) {
        Serial.read(); // Consuma il carattere

        // Spegni canale precedente
        if (currentChannel > 0) {
            pwm.setPWM(currentChannel - 1, 0, 0);
        }

        if (currentChannel < 16) {
            Serial.print("Testing channel ");
            Serial.print(currentChannel);
            Serial.println(" - PWM 50%");

            // Attiva canale corrente al 50%
            pwm.setPWM(currentChannel, 0, 2048);

            delay(2000);

            // Spegni
            pwm.setPWM(currentChannel, 0, 0);
            Serial.println("  -> Spento");

            currentChannel++;

            if (currentChannel < 16) {
                Serial.println("Premi un tasto per il prossimo canale...");
            } else {
                Serial.println("\n=== TEST COMPLETATO ===");
                Serial.println("Premi 'r' per ricominciare");
            }
        }
    }

    // Reset test
    if (Serial.available() && Serial.peek() == 'r') {
        Serial.read();
        currentChannel = 0;
        Serial.println("\n=== RICOMINCIAMO ===");
        Serial.println("Premi un tasto per testare canale 0...");
    }
}
