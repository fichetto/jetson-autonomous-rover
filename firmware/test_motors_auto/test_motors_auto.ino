/*
 * Test direzione motori per movimento AVANTI
 * Tutti i motori insieme - verifica che il rover avanzi
 */

#include <Wire.h>
#include "FaBoPWM_PCA9685.h"

FaBoPWM faboPWM;

// Canali PCA9685 - MAPPING CORRETTO
#define FL_IN1 2    // Front Left
#define FL_IN2 3
#define FR_IN1 0    // Front Right
#define FR_IN2 1
#define RL_IN1 6    // Rear Left
#define RL_IN2 7
#define RR_IN1 4    // Rear Right
#define RR_IN2 5

int PWM_POWER = 2000;  // Potenza massima

void setup() {
  Serial.begin(9600);
  Serial.println("=== TEST AVANTI/INDIETRO ===");

  if(faboPWM.begin()) {
    Serial.println("PCA9685 OK!");
    faboPWM.init(300);
    faboPWM.set_hz(50);
  } else {
    Serial.println("ERRORE PCA9685!");
    while(1);
  }

  stopAll();
  Serial.println("Test in 3 sec...");
  delay(3000);
}

void loop() {
  // TEST AVANTI - 3 secondi
  Serial.println("\n>>> AVANTI (3 sec)");
  Serial.println("    FL e RL: IN1=PWM, IN2=0");
  Serial.println("    FR e RR: IN1=0, IN2=PWM");

  // Lato sinistro: forward
  faboPWM.set_channel_value(FL_IN1, PWM_POWER);
  faboPWM.set_channel_value(FL_IN2, 0);
  faboPWM.set_channel_value(RL_IN1, PWM_POWER);
  faboPWM.set_channel_value(RL_IN2, 0);

  // Lato destro: forward (invertito meccanicamente)
  faboPWM.set_channel_value(FR_IN1, 0);
  faboPWM.set_channel_value(FR_IN2, PWM_POWER);
  faboPWM.set_channel_value(RR_IN1, 0);
  faboPWM.set_channel_value(RR_IN2, PWM_POWER);

  delay(3000);
  stopAll();
  Serial.println("    STOP");
  delay(3000);

  // TEST INDIETRO - 3 secondi
  Serial.println("\n>>> INDIETRO (3 sec)");

  // Lato sinistro: backward
  faboPWM.set_channel_value(FL_IN1, 0);
  faboPWM.set_channel_value(FL_IN2, PWM_POWER);
  faboPWM.set_channel_value(RL_IN1, 0);
  faboPWM.set_channel_value(RL_IN2, PWM_POWER);

  // Lato destro: backward
  faboPWM.set_channel_value(FR_IN1, PWM_POWER);
  faboPWM.set_channel_value(FR_IN2, 0);
  faboPWM.set_channel_value(RR_IN1, PWM_POWER);
  faboPWM.set_channel_value(RR_IN2, 0);

  delay(3000);
  stopAll();
  Serial.println("    STOP");
  delay(3000);

  Serial.println("\n===== RIPETO =====");
}

void stopAll() {
  faboPWM.set_channel_value(FL_IN1, 0);
  faboPWM.set_channel_value(FL_IN2, 0);
  faboPWM.set_channel_value(FR_IN1, 0);
  faboPWM.set_channel_value(FR_IN2, 0);
  faboPWM.set_channel_value(RL_IN1, 0);
  faboPWM.set_channel_value(RL_IN2, 0);
  faboPWM.set_channel_value(RR_IN1, 0);
  faboPWM.set_channel_value(RR_IN2, 0);
}
