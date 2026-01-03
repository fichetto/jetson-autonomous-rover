/*
 * I2C Scanner - Trova dispositivi sul bus I2C
 */

#include <Wire.h>

void setup() {
    Wire.begin();
    Serial.begin(115200);
    while (!Serial);
    Serial.println("\nI2C Scanner");
}

void loop() {
    byte error, address;
    int nDevices = 0;

    Serial.println("Scanning...");

    for (address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0) {
            Serial.print("I2C device found at address 0x");
            if (address < 16) Serial.print("0");
            Serial.print(address, HEX);

            if (address == 0x40) Serial.print(" (PCA9685)");
            if (address == 0x41) Serial.print(" (PCA9685 #2)");
            if (address == 0x70) Serial.print(" (PCA9685 All-Call)");

            Serial.println();
            nDevices++;
        }
    }

    if (nDevices == 0)
        Serial.println("No I2C devices found\n");
    else
        Serial.println("Done\n");

    delay(5000);
}
