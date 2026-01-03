/*
 * Test SUPER SEMPLICE per motori Moebius
 * Basato sul codice originale MoebiusTech
 *
 * Comandi seriali (9600 baud):
 *   1 = Motor A forward
 *   2 = Motor B forward
 *   3 = Motor C forward
 *   4 = Motor D forward
 *   A = AVANTI (tutti)
 *   B = INDIETRO (tutti)
 *   Z = STOP
 */

#include "FaBoPWM_PCA9685.h"

FaBoPWM faboPWM;

// Canali motori (come da Moebius originale)
#define DIRA1 0
#define DIRA2 1
#define DIRB1 2
#define DIRB2 3
#define DIRC1 4
#define DIRC2 5
#define DIRD1 6
#define DIRD2 7

// PWM (come da Moebius: range 300-2000)
int Motor_PWM = 1500;

// Macro controllo motori (ESATTAMENTE come Moebius)
#define MOTORA_FORWARD(pwm)    do{faboPWM.set_channel_value(DIRA1,pwm);faboPWM.set_channel_value(DIRA2, 0);}while(0)
#define MOTORA_STOP(x)         do{faboPWM.set_channel_value(DIRA1,0);faboPWM.set_channel_value(DIRA2, 0);}while(0)
#define MOTORA_BACKOFF(pwm)    do{faboPWM.set_channel_value(DIRA1,0);faboPWM.set_channel_value(DIRA2, pwm);}while(0)

#define MOTORB_FORWARD(pwm)    do{faboPWM.set_channel_value(DIRB1,pwm);faboPWM.set_channel_value(DIRB2, 0);}while(0)
#define MOTORB_STOP(x)         do{faboPWM.set_channel_value(DIRB1,0);faboPWM.set_channel_value(DIRB2, 0);}while(0)
#define MOTORB_BACKOFF(pwm)    do{faboPWM.set_channel_value(DIRB1,0);faboPWM.set_channel_value(DIRB2, pwm);}while(0)

#define MOTORC_FORWARD(pwm)    do{faboPWM.set_channel_value(DIRC1,pwm);faboPWM.set_channel_value(DIRC2, 0);}while(0)
#define MOTORC_STOP(x)         do{faboPWM.set_channel_value(DIRC1,0);faboPWM.set_channel_value(DIRC2, 0);}while(0)
#define MOTORC_BACKOFF(pwm)    do{faboPWM.set_channel_value(DIRC1,0);faboPWM.set_channel_value(DIRC2, pwm);}while(0)

#define MOTORD_FORWARD(pwm)    do{faboPWM.set_channel_value(DIRD1,pwm);faboPWM.set_channel_value(DIRD2, 0);}while(0)
#define MOTORD_STOP(x)         do{faboPWM.set_channel_value(DIRD1,0);faboPWM.set_channel_value(DIRD2, 0);}while(0)
#define MOTORD_BACKOFF(pwm)    do{faboPWM.set_channel_value(DIRD1,0);faboPWM.set_channel_value(DIRD2, pwm);}while(0)

void STOP_ALL() {
  MOTORA_STOP(0);
  MOTORB_STOP(0);
  MOTORC_STOP(0);
  MOTORD_STOP(0);
}

void AVANTI() {
  // Come da Moebius: A e C backoff, B e D forward
  MOTORA_BACKOFF(Motor_PWM);
  MOTORB_FORWARD(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM);
  MOTORD_FORWARD(Motor_PWM);
}

void INDIETRO() {
  MOTORA_FORWARD(Motor_PWM);
  MOTORB_BACKOFF(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM);
  MOTORD_BACKOFF(Motor_PWM);
}

void setup() {
  Serial.begin(9600);

  Serial.println("=== TEST MOEBIUS ===");

  if(faboPWM.begin()) {
    Serial.println("PCA9685 trovato!");
    faboPWM.init(300);  // Come Moebius
  } else {
    Serial.println("ERRORE: PCA9685 non trovato!");
  }

  faboPWM.set_hz(50);  // Come Moebius

  STOP_ALL();

  Serial.println("Comandi:");
  Serial.println("  1-4 = Motor A/B/C/D singoli");
  Serial.println("  A = Avanti, B = Indietro, Z = Stop");
  Serial.println("Pronto!");
}

void loop() {
  if(Serial.available()) {
    char cmd = Serial.read();

    switch(cmd) {
      case '1':
        Serial.println("Motor A forward...");
        MOTORA_FORWARD(Motor_PWM);
        delay(2000);
        MOTORA_STOP(0);
        Serial.println("Stop");
        break;

      case '2':
        Serial.println("Motor B forward...");
        MOTORB_FORWARD(Motor_PWM);
        delay(2000);
        MOTORB_STOP(0);
        Serial.println("Stop");
        break;

      case '3':
        Serial.println("Motor C forward...");
        MOTORC_FORWARD(Motor_PWM);
        delay(2000);
        MOTORC_STOP(0);
        Serial.println("Stop");
        break;

      case '4':
        Serial.println("Motor D forward...");
        MOTORD_FORWARD(Motor_PWM);
        delay(2000);
        MOTORD_STOP(0);
        Serial.println("Stop");
        break;

      case 'A':
      case 'a':
        Serial.println("AVANTI...");
        AVANTI();
        delay(2000);
        STOP_ALL();
        Serial.println("Stop");
        break;

      case 'B':
      case 'b':
        Serial.println("INDIETRO...");
        INDIETRO();
        delay(2000);
        STOP_ALL();
        Serial.println("Stop");
        break;

      case 'Z':
      case 'z':
        Serial.println("STOP!");
        STOP_ALL();
        break;
    }
  }
}
