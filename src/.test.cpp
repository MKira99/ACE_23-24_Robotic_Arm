#include <Arduino.h>
#include <EEPROM.h>
#include "RP2040_PWM.h"
#include "commands.h"
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <VL53L0X.h>

#define Servo1_PIN 0
#define Servo2_PIN 1
#define Servo3_PIN 2
#define Servo4_PIN 3

#define MAXSERVO1 13.2 //180 graus
#define MINSERVO1 4

#define MAXSERVO2_MOVE 4.9
#define MAXSERVO2 3.1
#define MINSERVO2 1.9

#define MAXSERVO3 6.5
#define MINSERVO3 3.7

#define MAXSERVO4 11.5
#define MINSERVO4 2

void setup(){

RP2040_PWM* PWM_Instance1;
RP2040_PWM* PWM_Instance2;
RP2040_PWM* PWM_Instance3;
RP2040_PWM* PWM_Instance4;

float PWM_frequency = 50;
PWM_Instance1 = new RP2040_PWM(Servo1_PIN, PWM_frequency, 0);
PWM_Instance2 = new RP2040_PWM(Servo2_PIN, PWM_frequency, 0);
PWM_Instance3 = new RP2040_PWM(Servo3_PIN, PWM_frequency, 0);
PWM_Instance4 = new RP2040_PWM(Servo4_PIN, PWM_frequency, 0);

PWM_Instance1->setPWM(Servo1_PIN, PWM_frequency, (61/100.0)*(MAXSERVO1-MINSERVO1)+MINSERVO1);
PWM_Instance2->setPWM(Servo2_PIN, PWM_frequency, MAXSERVO2_MOVE);
PWM_Instance3->setPWM(Servo3_PIN, PWM_frequency, MAXSERVO3);
PWM_Instance4->setPWM(Servo4_PIN, PWM_frequency, 11.5);

}