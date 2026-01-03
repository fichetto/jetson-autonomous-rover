/*
 * Test minimo Modbus RTU - senza PCA9685
 */

#define INPUT_REGISTER_NUM 25
#define HOLDING_REGISTER_NUM 20
#include <ModbusRTU.h>

ModbusRTUServer modbusServer;

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);  // LED on during setup

    // Initialize Modbus RTU Server
    modbusServer.startModbusServer(1, 115200, Serial, true);

    // Set some test values
    modbusServer.setHoldingValue(0, 255);  // Motor FL = stop
    modbusServer.setHoldingValue(1, 255);  // Motor FR = stop
    modbusServer.setHoldingValue(2, 255);  // Motor RL = stop
    modbusServer.setHoldingValue(3, 255);  // Motor RR = stop

    modbusServer.setInputValue(20, 11500);  // Fake battery 11.5V

    digitalWrite(LED_BUILTIN, LOW);  // LED off when ready
}

void loop() {
    bool writeOccurred = modbusServer.communicationLoop();

    if (writeOccurred) {
        // Blink LED on write
        digitalWrite(LED_BUILTIN, HIGH);
        delay(50);
        digitalWrite(LED_BUILTIN, LOW);
    }
}
