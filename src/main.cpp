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

VL53L0X tof;
float distance, prev_distance;
RP2040_PWM* PWM_Instance1;
RP2040_PWM* PWM_Instance2;
float PWM_frequency;
float Servo1_DC=0,Servo2_DC=100;
unsigned long interval, last_cycle;
unsigned long loop_micros;
commands_t serial_commands;
int show_lux;

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_1X);

typedef enum{
  WAIT,
  PICK_UP,
  READ_COLOUR,
  SORT,
}ROBOTIC_ARM;

typedef enum{
  WAIT_MOVE,
  SWEEP,
  SERVOS_MOVE,
  CLAW_CLOSE,
  CLAW_OPEN,
  FINISH,
}MOVE;

ROBOTIC_ARM cs_robotic_arm=WAIT,lastcs_robotic_arm=WAIT;
MOVE cs_move=WAIT_MOVE;

unsigned long SERVOS_MOVE_TIMER,CLAW_CLOSE_TIMER,CLAW_OPEN_TIMER;
unsigned int MODE=1; //1 for Predefined | 2 for Grid | 3 for Unknown
bool PICK_UP_MODE=0; //1 if needed grabing
bool START=0;
bool FINISHED=0;
bool START_MOVE=0;
bool SWEEP_FINISHED=0;

void getRawData_noDelay(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c)
{
  *c = tcs.read16(TCS34725_CDATAL);
  *r = tcs.read16(TCS34725_RDATAL);
  *g = tcs.read16(TCS34725_GDATAL);
  *b = tcs.read16(TCS34725_BDATAL);
}

void setup(){
  Serial.begin(115200);

  PWM_frequency = 50;
  PWM_Instance1 = new RP2040_PWM(Servo1_PIN, PWM_frequency, Servo1_DC);
  PWM_Instance2 = new RP2040_PWM(Servo2_PIN, PWM_frequency, Servo2_DC);
                    
  Wire.setSDA(20);
  Wire.setSCL(21);
  

  Wire1.setSDA(18);
  Wire1.setSCL(19);
  
  Wire.begin();
  Wire1.begin();
  
  while (!tcs.begin()) {
    Serial.println("No TCS34725 found ... check your connections");
    delay(500);
  }
  tcs.begin();

  Serial.println("Found TCS34725 sensor");

  while (!tof.init()) {
    Serial.println(F("Failed to detect and initialize VL53L0X!"));
    delay(100);
  }
  tof.init();

  Serial.println("Found VL53L0X sensor");
  
  interval = 50;

  //tof.startReadRangeMillimeters();  
}

void loop(){
  ///*
  unsigned long now = millis();
  if (now - last_cycle > interval) 
  {
    loop_micros = micros();
    last_cycle = now;

    uint16_t r, g, b, c, colorTemp, lux;
    getRawData_noDelay(&r, &g, &b, &c);
    colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);

    if (tof.readRangeAvailable()) 
    {
    prev_distance = distance;
    distance = tof.readRangeMillimeters() * 1e-3;
    }
    tof.startReadRangeMillimeters();



    switch (cs_robotic_arm)
    {
    case WAIT:
      if(START==1)
      {
        START=0;
        START_MOVE=1;
        cs_robotic_arm=PICK_UP;
      }
    break;
    case PICK_UP:
      if(FINISHED==1)
      {
        FINISHED=0;
        START_MOVE=1;
        cs_robotic_arm=READ_COLOUR;
      }
    break;
    case READ_COLOUR:
      if(FINISHED==1)
      {
        FINISHED=0;
        START_MOVE=1;
        cs_robotic_arm=SORT;
      }
    break;
    case SORT:
      if(FINISHED==1)
      {
        FINISHED=0;
        START_MOVE=1;
        cs_robotic_arm=WAIT;
      }
    }
  
    switch (cs_move)
    {
    case WAIT_MOVE:
      if(START_MOVE==1)
      {
        if(cs_robotic_arm==PICK_UP && MODE==3)
        {
          cs_move=SWEEP;
        }
        else
        {
          lastcs_robotic_arm=cs_robotic_arm;
          START_MOVE=0;
          cs_move=SERVOS_MOVE;
          SERVOS_MOVE_TIMER=now;
        }
      }
      break;
    case SWEEP:
      if(SWEEP_FINISHED==1)
      {
        SWEEP_FINISHED=0;
        lastcs_robotic_arm=cs_robotic_arm;
        START_MOVE=0;
        cs_move=SERVOS_MOVE;
        SERVOS_MOVE_TIMER=now;
      }
      break;
    case SERVOS_MOVE:
      if(now-SERVOS_MOVE_TIMER>=500)
      {
        if(PICK_UP_MODE==1)
        {
          cs_move=CLAW_CLOSE;
          CLAW_CLOSE_TIMER=now;
        }
        else
        {
          cs_move=CLAW_OPEN;
          CLAW_OPEN_TIMER=now;
        }
      }
      break;
    case CLAW_CLOSE:
      if(now-CLAW_CLOSE_TIMER>=500)
      {
        cs_move=FINISH;
      }
      break;
    case CLAW_OPEN:
      if(now-CLAW_OPEN_TIMER>=500)
      {
        cs_move=FINISH;
      }
      break;
    case FINISH:
      if(lastcs_robotic_arm!=cs_robotic_arm)
      {
        cs_move=WAIT_MOVE;
      }

    
    default:
      break;
    }
  
    if(cs_move==SERVOS_MOVE)
    {
      if(cs_robotic_arm==PICK_UP && MODE==1)
      {

      }
      else if(cs_robotic_arm==PICK_UP && MODE==2)
      {

      }
      else if(cs_robotic_arm==PICK_UP && MODE==3)
      {

      }
      else if(cs_robotic_arm==READ_COLOUR)
      {

      }
      else if(cs_robotic_arm==SORT)
      {

      }
    }


    
  }//*/
    /*uint8_t b;
    if (Serial.available()) {
      
      b = Serial.read();    

      if (b == 'a'){
        Servo1_DC -= 0.1; 
      }
      else if (b == 'q'){
        Servo1_DC += 0.1;
      }
      else if (b == 's'){
        Servo2_DC -= 0.1;
      }
      else if (b == 'w'){
        Servo2_DC += 0.1;
      }
      else serial_commands.process_char(b);

      if (Servo1_DC > 13.2) Servo1_DC = 13.2;
      if (Servo1_DC < 2.90) Servo1_DC = 2.90;

      if (Servo2_DC > 20) Servo2_DC = 20;
      if (Servo2_DC < 0) Servo2_DC = 0;
    }
    unsigned long now = millis();
    if (now - last_cycle > interval) {
      loop_micros = micros();
      last_cycle = now;

      show_lux = 1;
      uint16_t r, g, b, c, colorTemp, lux;
      getRawData_noDelay(&r, &g, &b, &c);
      colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
      if (show_lux) lux = tcs.calculateLux(r, g, b);

      Serial.print("Color Temp: "); Serial.print(colorTemp, DEC); Serial.print(" K - ");
      if (show_lux) Serial.print("Lux: "); Serial.print(lux, DEC); Serial.print(" - ");
      Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
      Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
      Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
      Serial.print("C: "); Serial.print(c, DEC); Serial.print(" ");
      Serial.print("\n");
      
      // Update the LED intensity
      PWM_Instance1->setPWM(Servo1_PIN, PWM_frequency, Servo1_DC);
      PWM_Instance2->setPWM(Servo2_PIN, PWM_frequency, Servo2_DC);


      // Debug using the serial port
      Serial.print("DC Servo1 : ");
      Serial.print(Servo1_DC);
      Serial.print("\n");
      Serial.print("DC Servo2 : ");
      Serial.print(Servo2_DC);
      Serial.print("\n");

      if (tof.readRangeAvailable()) {
      prev_distance = distance;
      distance = tof.readRangeMillimeters() * 1e-3;
      }
      tof.startReadRangeMillimeters(); 
      Serial.print("Dist: ");
      Serial.print(distance, 3);
      Serial.println();
    }*/
}

