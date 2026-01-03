/*
 * Test Modbus RTU con libreria modbus-esp8266
 */

#include <ModbusRTU.h>

ModbusRTU mb;

// Holding registers (0-9)
#define HREG_MOTOR_FL 0
#define HREG_MOTOR_FR 1
#define HREG_MOTOR_RL 2
#define HREG_MOTOR_RR 3

// Input registers
#define IREG_BATTERY 0

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);  // LED on during setup

    Serial.begin(115200);

    // Initialize Modbus slave with ID 1
    mb.begin(&Serial);
    mb.slave(1);

    // Add holding registers (motor speeds)
    mb.addHreg(HREG_MOTOR_FL, 255);  // Default stop
    mb.addHreg(HREG_MOTOR_FR, 255);
    mb.addHreg(HREG_MOTOR_RL, 255);
    mb.addHreg(HREG_MOTOR_RR, 255);

    // Add input register (battery voltage)
    mb.addIreg(IREG_BATTERY, 11500);  // Fake 11.5V

    digitalWrite(LED_BUILTIN, LOW);  // LED off when ready
}

void loop() {
    mb.task();

    // Check if motor register changed, blink LED
    static uint16_t lastFL = 255;
    uint16_t currentFL = mb.Hreg(HREG_MOTOR_FL);
    if (currentFL != lastFL) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(50);
        digitalWrite(LED_BUILTIN, LOW);
        lastFL = currentFL;
    }

    yield();
}
