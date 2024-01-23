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

VL53L0X tof;
float distance, prev_distance;
RP2040_PWM* PWM_Instance1;
RP2040_PWM* PWM_Instance2;
RP2040_PWM* PWM_Instance3;
RP2040_PWM* PWM_Instance4;
float PWM_frequency;
float Servo1_DC=100,Servo2_DC=100,Servo3_DC=100,Servo4_DC=100;
unsigned long interval, last_cycle;
unsigned long loop_micros;
commands_t serial_commands;
int show_lux;
int counter=0;

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
  SERVOS_ROTATE,
  SERVOS_ADVANCE,
  CLAW,
  RETRACT,
  FINISH,
}MOVE;

typedef struct{
  float rot;
  float dist;
}SERVO_PRED_POS;

ROBOTIC_ARM cs_robotic_arm=WAIT,lastcs_robotic_arm=WAIT;
MOVE cs_move=WAIT_MOVE;

unsigned long SERVOS_ADVANCE_TIMER,SERVOS_ROTATE_TIMER,CLAW_TIMER,RETRACT_TIMER;
unsigned int MODE=1; //1 for Predefined | 2 for Grid | 3 for Unknown
unsigned int OP_MODE=1;
bool CLAW_CHANGE=0;
bool CLAW_OPEN_OR_CLOSE=1; // 0 for close | 1 for open
bool START=0;
bool FINISHED=0;
bool START_MOVE=0;
bool SWEEP_FINISHED=0;

unsigned int count=0;
bool count_aux=0;
unsigned int slow_rot=0, slow_dist=0;
unsigned int count_print=0;

SERVO_PRED_POS GRID[9]={{38,100},{38,50},{38,0},{61,100},{61,50},{61,0},{80,100},{80,50},{80,0}};
SERVO_PRED_POS PRED_POS={61,50};
SERVO_PRED_POS UNKNOWN_POS={0,0};
SERVO_PRED_POS READ_COLOUR_POS={100,100};
SERVO_PRED_POS SORT_GREEN_POS={5,0};
SERVO_PRED_POS SORT_YELLOW_POS={11,0};

unsigned int GOTOGRID=1;
unsigned int OP_GOTOGRID=1;
unsigned int COLOUR_READ=2;

void getRawData_noDelay(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c)
{
  *c = tcs.read16(TCS34725_CDATAL);
  *r = tcs.read16(TCS34725_RDATAL);
  *g = tcs.read16(TCS34725_GDATAL);
  *b = tcs.read16(TCS34725_BDATAL);
}

void servos_write(int DC1,int DC2,int DC3)
{
  Servo1_DC=(DC1/100.0)*(MAXSERVO1-MINSERVO1)+MINSERVO1;
  Servo2_DC=(DC2/100.0)*(MAXSERVO2-MINSERVO2)+MINSERVO2;
  Servo3_DC=(DC3/100.0)*(MAXSERVO3-MINSERVO3)+MINSERVO3;
  if(DC2==-1) Servo2_DC=MAXSERVO2_MOVE;
  if(DC3==-1) Servo3_DC=MAXSERVO3;

  PWM_Instance1->setPWM(Servo1_PIN, PWM_frequency, Servo1_DC);
  PWM_Instance2->setPWM(Servo2_PIN, PWM_frequency, Servo2_DC);
  PWM_Instance3->setPWM(Servo3_PIN, PWM_frequency, Servo3_DC);
}
void servos_init_pos()
{
  PWM_Instance1->setPWM(Servo1_PIN, PWM_frequency, (61/100.0)*(MAXSERVO1-MINSERVO1)+MINSERVO1);
  PWM_Instance2->setPWM(Servo2_PIN, PWM_frequency, MAXSERVO2_MOVE);
  PWM_Instance3->setPWM(Servo3_PIN, PWM_frequency, MAXSERVO3);
  PWM_Instance4->setPWM(Servo4_PIN, PWM_frequency, 11.5);
}

void setup(){
  Serial.begin(115200);
  Serial.print("Starting...\n");

  PWM_frequency = 50;
  PWM_Instance1 = new RP2040_PWM(Servo1_PIN, PWM_frequency, Servo1_DC);
  PWM_Instance2 = new RP2040_PWM(Servo2_PIN, PWM_frequency, Servo2_DC);
  PWM_Instance3 = new RP2040_PWM(Servo3_PIN, PWM_frequency, Servo3_DC);
  PWM_Instance4 = new RP2040_PWM(Servo4_PIN, PWM_frequency, Servo4_DC);

  PWM_Instance1->setPWM(Servo1_PIN, PWM_frequency, (61/100.0)*(MAXSERVO1-MINSERVO1)+MINSERVO1);
  PWM_Instance2->setPWM(Servo2_PIN, PWM_frequency, MAXSERVO2_MOVE);
  PWM_Instance3->setPWM(Servo3_PIN, PWM_frequency, MAXSERVO3);
  PWM_Instance4->setPWM(Servo4_PIN, PWM_frequency, 11.5);
                    
  Wire.setSDA(20);
  Wire.setSCL(21);

  Serial.print("Starting I2C...\n");
  

  Wire1.setSDA(18);
  Wire1.setSCL(19);

  Serial.print("Starting I2C1...\n");
  
  Wire.begin();
  Wire1.begin();

  Serial.print("Begin I2Cs...\n");
  
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
  
  interval = 20;

  //tof.startReadRangeMillimeters();  
}

void loop()
{
  unsigned long now = millis();
  if (now - last_cycle > interval) 
  {
    loop_micros = micros();
    last_cycle = now;

    uint8_t key;
    if (Serial.available()) {
      
      key = Serial.read();  

      if(key == 'z'){
        START=1;
      }
      else if (key == '1'){
        GOTOGRID=1;
        MODE=2;
      }
      else if (key == '2'){
        GOTOGRID=2;
        MODE=2;
      }
      else if (key == '3'){
        GOTOGRID=3;
        MODE=2;
      }
      else if(key == '4'){
        GOTOGRID=4;
        MODE=2;
      }
      else if(key == '5'){
        GOTOGRID=5;
        MODE=2;
      }
      else if(key == '6'){
        GOTOGRID=6;
        MODE=2;
      }
      else if(key == '7'){
        GOTOGRID=7;
        MODE=2;
      }
      else if(key == '8'){
        GOTOGRID=8;
        MODE=2;
      }
      else if(key == '9'){
        GOTOGRID=9;
        MODE=2;
      }
      else if(key == 'p')
      {
        MODE=1;
      }
    }
    count_print++;
    if(count_print>100)
    {
      Serial.print("Selected Mode: ");
      Serial.print(MODE);
      if(MODE==2)
      {
        Serial.print(" | Selected Grid: ");
        Serial.print(GOTOGRID);
      }
      Serial.print("\n");

      Serial.print("OP_Mode: ");
      Serial.print(OP_MODE);
      if(OP_MODE==2)
      {
        Serial.print(" | OP_GOTOGRID: ");
        Serial.print(OP_GOTOGRID);
      }
      Serial.print("\n");
      count_print=0;
    }
    

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
        lastcs_robotic_arm=SORT;
        cs_robotic_arm=PICK_UP;
        FINISHED=0;
        CLAW_CHANGE=1;
        OP_MODE=MODE;
        if(OP_MODE==2)
        {
          OP_GOTOGRID=GOTOGRID;
        }
      }
    break;
    case PICK_UP:
      if(FINISHED==1)
      {
        FINISHED=0;
        START_MOVE=1;
        lastcs_robotic_arm=cs_robotic_arm;
        cs_robotic_arm=READ_COLOUR;
      }
    break;
    case READ_COLOUR:
      if(FINISHED==1)
      {
        FINISHED=0;
        START_MOVE=1;
        lastcs_robotic_arm=cs_robotic_arm;
        cs_robotic_arm=SORT;
        CLAW_CHANGE=1;
      }
    break;
    case SORT:
      if(FINISHED==1)
      {
        FINISHED=0;
        lastcs_robotic_arm=cs_robotic_arm;
        cs_robotic_arm=WAIT;
      }
    }
  
    switch (cs_move)
    {
    case WAIT_MOVE:
      if(START_MOVE==1)
      {
        if(cs_robotic_arm==PICK_UP && OP_MODE==3)
        {
          cs_move=SWEEP;
        }
        else
        {
          lastcs_robotic_arm=cs_robotic_arm;
          START_MOVE=0;
          cs_move=SERVOS_ROTATE;
          SERVOS_ROTATE_TIMER=now;
        }
      }
      break;
    case SWEEP:
      if(SWEEP_FINISHED==1)
      {
        SWEEP_FINISHED=0;
        lastcs_robotic_arm=cs_robotic_arm;
        START_MOVE=0;
        cs_move=SERVOS_ROTATE;
        SERVOS_ROTATE_TIMER=now;
      }
      break;
    case SERVOS_ROTATE:
     if(now - SERVOS_ROTATE_TIMER >= 1000)
     {
      cs_move=SERVOS_ADVANCE;
      SERVOS_ADVANCE_TIMER=now;
     }
      
      break;
    case SERVOS_ADVANCE:
      if(now-SERVOS_ADVANCE_TIMER>=1000)
      {
        if(CLAW_CHANGE==1)
        {
          cs_move=CLAW;
          CLAW_TIMER=now;
          if(CLAW_OPEN_OR_CLOSE==1)
          {
            CLAW_OPEN_OR_CLOSE=0;
          }
          else
          {
            CLAW_OPEN_OR_CLOSE=1;
          }
          CLAW_CHANGE=0;
        }
        else
        {
          cs_move=RETRACT;
          RETRACT_TIMER=now;
        }
      }
      break;
    case CLAW:
      if(now-CLAW_TIMER>=500)
      {
        RETRACT_TIMER=now;
        cs_move=RETRACT;
      }
      break;
    case RETRACT:
      if(now-RETRACT_TIMER>=1000)
      {
        FINISHED=1;
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
  
    if(cs_move==SERVOS_ROTATE)
    {
      if(cs_robotic_arm==PICK_UP && OP_MODE==1)
      {
        Serial.print("1\n");
        servos_write(PRED_POS.rot,-1,-1);
      }
      else if(cs_robotic_arm==PICK_UP && OP_MODE==2)
      {
        servos_write(GRID[OP_GOTOGRID-1].rot,-1,-1);
      }
      else if(cs_robotic_arm==PICK_UP && OP_MODE==3)
      {
        servos_write(UNKNOWN_POS.rot,-1,-1);
      }
      else if(cs_robotic_arm==READ_COLOUR)
      {
        servos_write(READ_COLOUR_POS.rot,-1,-1);
      }
      else if(cs_robotic_arm==SORT)
      {
        if(COLOUR_READ==1)
        {
          Serial.print("YELLOW\n");
          servos_write(SORT_YELLOW_POS.rot,-1,-1);
        }
        else if(COLOUR_READ==2)
        {
          Serial.print("GREEN\n");
          servos_write(SORT_GREEN_POS.rot,-1,-1);
        }
      }
    }
    else if(cs_move==SERVOS_ADVANCE)
    {
      if(cs_robotic_arm==PICK_UP && OP_MODE==1)
      {
        Serial.print("1\n");
        servos_write(PRED_POS.rot,PRED_POS.dist,PRED_POS.dist);
      }
      else if(cs_robotic_arm==PICK_UP && OP_MODE==2)
      {
        servos_write(GRID[OP_GOTOGRID-1].rot,GRID[OP_GOTOGRID-1].dist,GRID[OP_GOTOGRID-1].dist);
      }
      else if(cs_robotic_arm==PICK_UP && OP_MODE==3)
      {
        servos_write(UNKNOWN_POS.rot,UNKNOWN_POS.dist,UNKNOWN_POS.dist);
      }
      else if(cs_robotic_arm==READ_COLOUR)
      {
        servos_write(READ_COLOUR_POS.rot,READ_COLOUR_POS.dist,READ_COLOUR_POS.dist);
      }
      else if(cs_robotic_arm==SORT)
      {
        if(COLOUR_READ==1)
        {
          Serial.print("YELLOW\n");
          servos_write(SORT_YELLOW_POS.rot,SORT_YELLOW_POS.dist,SORT_YELLOW_POS.dist);
        }
        else if(COLOUR_READ==2)
        {
          Serial.print("GREEN\n");
          servos_write(SORT_GREEN_POS.rot,SORT_GREEN_POS.dist,SORT_GREEN_POS.dist);
        }
      }
    }
    else if(cs_move==RETRACT)
    {
      if(cs_robotic_arm==PICK_UP && OP_MODE==1)
      {
        Serial.print("1\n");
        servos_write(PRED_POS.rot,-1,-1);
      }
      else if(cs_robotic_arm==PICK_UP && OP_MODE==2)
      {
        servos_write(GRID[OP_GOTOGRID-1].rot,-1,-1);
      }
      else if(cs_robotic_arm==PICK_UP && OP_MODE==3)
      {
        servos_write(UNKNOWN_POS.rot,-1,-1);
      }
      else if(cs_robotic_arm==SORT)
      {
        if(COLOUR_READ==1)
        {
          Serial.print("YELLOW\n");
          servos_write(SORT_YELLOW_POS.rot,-1,-1);
        }
        else if(COLOUR_READ==2)
        {
          Serial.print("GREEN\n");
          servos_write(SORT_GREEN_POS.rot,-1,-1);
        }
      }
    }
    else if(cs_move==CLAW)
    {
      if(CLAW_OPEN_OR_CLOSE==1)
      {
        Serial.print("OPEN\n");
        PWM_Instance4->setPWM(Servo4_PIN, PWM_frequency, 11.5);
      }
      else
      {
        Serial.print("CLOSE\n");
        PWM_Instance4->setPWM(Servo4_PIN, PWM_frequency, 2);
      }
    }
    
    if(cs_robotic_arm==WAIT)
    {
      servos_init_pos();
    }
    
  }
    
    
    /*
    uint8_t b;
    if (Serial.available()) {
      
      b = Serial.read();    

      if (b == 'a'){
        Servo1_DC -= 0.1;
        exemple1.rot-=1;
      }
      else if (b == 'q'){
        Servo1_DC += 0.1;
        exemple1.rot+=1;
      }
      else if (b == 's'){
        Servo2_DC -= 0.1;
        exemple1.dist-=1;
      }
      else if (b == 'w'){
        Servo2_DC += 0.1;
        exemple1.dist+=1;
      }
      else if (b == 'd'){
        Servo3_DC -= 0.1; 
      }
      else if (b == 'e'){
        Servo3_DC += 0.1;
      }
      else if(b == 'f'){
        Servo4_DC -= 1;
      }
      else if(b == 'r'){
        Servo4_DC += 1;
      }
      else if (b == '1'){
        GOTOGRID=1;
      }
      else if (b == '2'){
        GOTOGRID=2;
      }
      else if (b == '3'){
        GOTOGRID=3;
      }
      else if(b == '4'){
        GOTOGRID=4;
      }
      else if(b == '5'){
        GOTOGRID=5;
      }
      else if(b == '6'){
        GOTOGRID=6;
      }
      else if(b == '7'){
        GOTOGRID=7;
      }
      else if(b == '8'){
        GOTOGRID=8;
      }
      else if(b == '9'){
        GOTOGRID=9;
      }
      else if(b == 'z'){
        GOTOGRID=0;
      }


      else serial_commands.process_char(b);

      if (Servo1_DC > MAXSERVO1) Servo1_DC = MAXSERVO1;
      if (Servo1_DC < MINSERVO1) Servo1_DC = MINSERVO1;

      if (Servo2_DC > MAXSERVO2_MOVE) Servo2_DC = MAXSERVO2_MOVE;
      if (Servo2_DC < MINSERVO2) Servo2_DC = MINSERVO2;

      if (Servo3_DC > MAXSERVO3) Servo3_DC = MAXSERVO3;
      if (Servo3_DC < MINSERVO3) Servo3_DC = MINSERVO3;

      //if (Servo4_DC > MAXSERVO4) Servo4_DC = MAXSERVO4;
      //if (Servo4_DC < MINSERVO4) Servo4_DC = MINSERVO4;

      if(exemple1.rot>100) exemple1.rot=100;
      if(exemple1.rot<0) exemple1.rot=0;
      if(exemple1.dist>100) exemple1.dist=100;
      if(exemple1.dist<0) exemple1.dist=0;
      if(Servo4_DC>100) Servo4_DC=100;
      if(Servo4_DC<0) Servo4_DC=0;
      
    }
    unsigned long now = millis();
    if (now - last_cycle > interval) {
      loop_micros = micros();
      last_cycle = now;
      
      /*
      show_lux = 1;
      uint16_t r, g, b, c, colorTemp, lux;
      getRawData_noDelay(&r, &g, &b, &c);
      colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
      if (show_lux) lux = tcs.calculateLux(r, g, b);
      */

      /*Serial.print("Color Temp: "); Serial.print(colorTemp, DEC); Serial.print(" K - ");
      if (show_lux) Serial.print("Lux: "); Serial.print(lux, DEC); Serial.print(" - ");
      Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
      Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
      Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
      Serial.print("C: "); Serial.print(c, DEC); Serial.print(" ");
      Serial.print("\n");*/
      
      // Update the LED intensity
      /*
      PWM_Instance1->setPWM(Servo1_PIN, PWM_frequency, Servo1_DC);
      PWM_Instance2->setPWM(Servo2_PIN, PWM_frequency, Servo2_DC);
      PWM_Instance3->setPWM(Servo3_PIN, PWM_frequency, Servo3_DC);
      PWM_Instance4->setPWM(Servo4_PIN, PWM_frequency, Servo4_DC);


      Serial.print("Rotaçao : ");
      Serial.print(slow_rot);
      Serial.print("\n");
      Serial.print("Distancia : ");
      Serial.print(slow_dist);
      Serial.print("\n");
      Serial.print("Pincas : ");
      Serial.print(Servo4_DC);
      Serial.print("\n");
      */

      //slow_dist=GRID[GOTOGRID-1].dist;
      //slow_rot=GRID[GOTOGRID-1].rot;
      /*
      if(GOTOGRID==0)
      {
        servos_init_pos();
      }
      else
      {
        if(slow_rot<GRID[GOTOGRID-1].rot)
        {
          slow_rot+=5;
        }
        else if(slow_rot>GRID[GOTOGRID-1].rot)
        {
          slow_rot-=5;
        }
        if(slow_rot<GRID[GOTOGRID-1].rot+20 && slow_rot>GRID[GOTOGRID-1].rot-20)
        {
          slow_rot=GRID[GOTOGRID-1].rot;
        }
        if(slow_dist<GRID[GOTOGRID-1].dist)
        {
          slow_dist+=5;
        }
        else if(slow_dist>GRID[GOTOGRID-1].dist)
        {
          slow_dist-=5;
        }
        if(slow_dist<GRID[GOTOGRID-1].dist+10 && slow_dist>GRID[GOTOGRID-1].dist-10)
        {
          slow_dist=GRID[GOTOGRID-1].dist;
        }
        servos_write(slow_rot,slow_dist,slow_dist,Servo4_DC);
      }
      */
      /*
      Serial.print("Rotaçao : ");
      Serial.print(exemple1.rot);
      Serial.print("\n");
      Serial.print("Distancia : ");
      Serial.print(exemple1.dist);
      Serial.print("\n");
      Serial.print("Pincas : ");
      Serial.print(Servo4_DC);
      Serial.print("\n");
      servos_write(exemple1.rot,exemple1.dist,exemple1.dist,Servo4_DC);
      */
      /*

      Serial.print("Timer: ");
      Serial.print(loop_micros);
      Serial.print("\n");


      
      Serial.print("DC Servo1 : ");
      Serial.print(Servo1_DC);
      Serial.print("\n");
      Serial.print("DC Servo2 : ");
      Serial.print(Servo2_DC);
      Serial.print("\n");
      Serial.print("DC Servo3 : ");
      Serial.print(Servo3_DC);
      Serial.print("\n");
      Serial.print("DC Servo4    : ");
      Serial.print(Servo4_DC);
      Serial.print("\n");
      */

      /*if (tof.readRangeAvailable()) {
      prev_distance = distance;
      distance = tof.readRangeMillimeters() * 1e-3;
      }
      tof.startReadRangeMillimeters(); 
      Serial.print("Dist: ");
      Serial.print(distance, 3);
      Serial.println();
    }
    */
}

