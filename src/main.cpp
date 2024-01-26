#include <Arduino.h>
#include <EEPROM.h>
#include "RP2040_PWM.h"
#include "commands.h"
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <VL53L0X.h>
#include <cmath>


#define Servo1_PIN 0
#define Servo2_PIN 1
#define Servo3_PIN 2
#define Servo4_PIN 3


#define MAXSERVO1 11.3 //180 graus
#define MINSERVO1 2.9

#define MAXSERVO2 11
#define MINSERVO2 8.9

#define MAXSERVO3 11
#define MINSERVO3 8.8

#define MAXSERVO4 10
#define MINSERVO4 2

#define DIST_MAX 0.21
#define DIST_MIN 0.11

#define WAITING_TIME 400


/*#define MAXSERVO1 20 //180 graus
#define MINSERVO1 0

#define MAXSERVO2 20
#define MINSERVO2 0

#define MAXSERVO3 20
#define MINSERVO3 0

#define MAXSERVO4 20
#define MINSERVO4 0*/



VL53L0X tof;
float distance, prev_distance;
RP2040_PWM* PWM_Instance1;
RP2040_PWM* PWM_Instance2;
RP2040_PWM* PWM_Instance3;
RP2040_PWM* PWM_Instance4;
float PWM_frequency;
float Servo1_DC=9,Servo2_DC=9,Servo3_DC=9,Servo4_DC=6;
unsigned long interval, last_cycle, last_cycle2;
unsigned long loop_micros;
commands_t serial_commands;
int show_lux;
int counter=0;

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_1X);

typedef enum{
  STANDBY,
  OPERATION,
  CONFIG,
  FLASH,
}MAIN;

typedef enum{
  WAIT,
  PICK_UP,
  READ_COLOUR,
  SORT,
}ROBOTIC_ARM;

typedef enum{
  WAIT_MOVE,
  SWEEP_WAIT,
  SWEEP,
  SWEEP_RETRACT,
  WAITING_GRID_CLEAR,
  SERVOS_ROTATE,
  SERVOS_ADVANCE,
  CLAW,
  RETRACT,
  WAITING_GRID_RESTORE,
  FINISH,
}MOVE;

typedef enum{
  WAIT_GRID,
  ROT_147,
  ADV_147,
  PICK_147,
  RETRACT_147,
  ROT_RETRACT_147,
  PUT_147,
  OPEN_CLAW_147,
  PUT_RETRACT_147,
  ROT_258,
  ADV_258,
  PICK_258,
  RETRACT_258,
  ROT_RETRACT_258,
  PUT_258,
  OPEN_CLAW_258,
  PUT_RETRACT_258,
}GRID_CLEAR;

typedef enum{
  WAIT_GRID_RESTORE,
  ROT_258_RESTORE,
  ADV_258_RESTORE,
  PICK_258_RESTORE,
  RETRACT_258_RESTORE,
  ROT_RETRACT_258_RESTORE,
  PUT_258_RESTORE,
  OPEN_CLAW_258_RESTORE,
  PUT_RETRACT_258_RESTORE,
  ROT_147_RESTORE,
  ADV_147_RESTORE,
  PICK_147_RESTORE,
  RETRACT_147_RESTORE,
  ROT_RETRACT_147_RESTORE,
  PUT_147_RESTORE,
  OPEN_CLAW_147_RESTORE,
  PUT_RETRACT_147_RESTORE,
}GRID_RESTORE;

typedef enum{
  WAIT_CONFIG,
  PRED_POS_CONFIG,
  GRID_POS_CONFIG,
  SORT_POS_CONFIG,
}CONFIG_TYPE;

typedef enum{
  WAIT_GRID_TYPE,
  GRID1,
  GRID2,
  GRID3,
  GRID4,
  GRID5,
  GRID6,
  GRID7,
  GRID8,
  GRID9,
}GRID_TYPE;

typedef enum{
  WAIT_COLOUR_TYPE,
  RED,
  GREEN,
  BLUE,
  YELLOW,
  MAGENTA,
  CYAN,
}COLOUR_TYPE;

typedef struct{
  unsigned int rot;
  unsigned int dist;
}SERVO_PRED_POS;

MAIN cs_main=STANDBY;
ROBOTIC_ARM cs_robotic_arm=WAIT,lastcs_robotic_arm=WAIT;
MOVE cs_move=WAIT_MOVE;
GRID_CLEAR cs_grid_clear=WAIT_GRID;
GRID_RESTORE cs_grid_restore=WAIT_GRID_RESTORE;
CONFIG_TYPE cs_config=WAIT_CONFIG;
GRID_TYPE cs_grid_type=WAIT_GRID_TYPE;
COLOUR_TYPE cs_colour_type=WAIT_COLOUR_TYPE;

unsigned long SERVOS_ADVANCE_TIMER,SERVOS_ROTATE_TIMER,CLAW_TIMER,RETRACT_TIMER,SWEEP_TIMER,GRID_CLEAR_TIMER;
unsigned int MODE=1; //1 for Predefined | 2 for Grid | 3 for Unknown
unsigned int OP_MODE=1;
unsigned int MAIN_MODE=1; //1 for Standby | 2 for Operation | 3 for Config
unsigned int OP_MAIN_MODE=1;
bool CLAW_CHANGE=0;
bool CLAW_OPEN_OR_CLOSE=1; // 0 for close | 1 for open
bool START=0;
bool FINISHED=0;
bool START_MOVE=0;
bool SWEEP_FINISHED=0;
float SWEEP_COUNT[100]={0};
int SWEEP_COUNT_AUX=0;
bool SWEEP_START=0;
bool GRID_FINISHED=0;
bool GRID_RESTORE_FINISHED=0;
bool CONFIG_SWITCH=0;
bool CONFIG_TYPES_SWITCH=0;
bool SAVE_CONFIG=0;
bool SWEEP_FAILED=0;
bool FIRST_SWEEP=1;

unsigned int count=0;
bool count_aux=0, sort_test_aux=0, read_colour_aux=0;
unsigned int slow_rot=0, slow_dist=0;
unsigned int count_print=0;
unsigned int config_dist=50, config_rot=50;

SERVO_PRED_POS GRID[9]={{40,20},{40,60},{40,100},{70,20},{70,70},{70,100},{90,20},{90,60},{90,100}};
SERVO_PRED_POS PRED_POS={50,50};
SERVO_PRED_POS SORT_RED_POS={10,100};
SERVO_PRED_POS SORT_GREEN_POS={10,100};
SERVO_PRED_POS SORT_BLUE_POS={10,100};
SERVO_PRED_POS SORT_YELLOW_POS={20,100};
SERVO_PRED_POS SORT_MAGENTA_POS={20,100};
SERVO_PRED_POS SORT_CYAN_POS={20,100};
SERVO_PRED_POS SORT_UNDETECTED_POS={100,100};

SERVO_PRED_POS UNKNOWN_POS={50,50};

unsigned int GOTOGRID=1;
unsigned int OP_GOTOGRID=1;
unsigned int COLOUR_READ=0;

// RGB -> color mapping data
const float distinctRGB[6][3] = {{255,0,0},{0,255,0},{0,0,255},{255,255,0},{255,0,255},{0,255,255}};
const String distinctColors[7] = {"red","green","blue","yellow","magenta","cyan","NA"};

String closestColor(int r, int g, int b) {
  String colorReturn = "NA";
  //float max_color = r+g+b;
  float max_color=max(r,max(g,b));
  /*Serial.print("MAX COLOR: ");
  Serial.print(max_color);
  Serial.print("\n");*/
  
  float biggestDifference = 1000;
  for (int i = 0; i < 6; i++) {
    /*Serial.print("Color Being Compared: ");
    Serial.print(distinctColors[i]);
    Serial.print("\n");*/
    float aux;
    //aux= distinctRGB[i][0] + distinctRGB[i][1] + distinctRGB[i][2];
    aux = max(distinctRGB[i][0], max(distinctRGB[i][1], distinctRGB[i][2]));
    float difference = sqrt(pow(r/max_color - distinctRGB[i][0]/aux, 2) + pow(g/max_color - distinctRGB[i][1]/aux, 2) + pow(b/max_color - distinctRGB[i][2]/aux, 2));
    /*Serial.print("DIFFERENCE: ");
    Serial.print(difference);
    Serial.print("\n");
    Serial.print("DISTINCT COLOR R : ");
    Serial.print(pow(r/max_color - distinctRGB[i][0]/aux, 2));
    Serial.print(" G : ");
    Serial.print(pow(g/max_color - distinctRGB[i][1]/aux, 2));
    Serial.print(" B : ");
    Serial.print(pow(b/max_color - distinctRGB[i][2]/aux, 2));
    Serial.print("\n");
    Serial.print("Colors Received R : ");
    Serial.print(r/max_color);
    Serial.print(" G : ");
    Serial.print(g/max_color);
    Serial.print(" B : ");
    Serial.print(b/max_color);
    Serial.print("\n");
    Serial.print("Colors Distinct R : ");
    Serial.print(distinctRGB[i][0]/aux);
    Serial.print(" G : ");
    Serial.print(distinctRGB[i][1]/aux);
    Serial.print(" B : ");
    Serial.print(distinctRGB[i][2]/aux);
    Serial.print("\n");
    Serial.print("\n");
    Serial.print("\n");
    Serial.print("\n");*/



    if (difference < biggestDifference) {
      colorReturn = distinctColors[i];
      biggestDifference = difference;
    }
  }
  if(biggestDifference>1)
  {
    colorReturn="NA";
    Serial.print("COLOR NOT DETECTED\n OBTAINTED COLOR: ");
    Serial.print(colorReturn);
    Serial.print("\n");
  }
  return colorReturn;
}

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
  if(DC2==-1) Servo2_DC=MINSERVO2;
  if(DC3==-1) Servo3_DC=MINSERVO3;

  PWM_Instance1->setPWM(Servo1_PIN, PWM_frequency, Servo1_DC);
  PWM_Instance2->setPWM(Servo2_PIN, PWM_frequency, Servo2_DC);
  PWM_Instance3->setPWM(Servo3_PIN, PWM_frequency, Servo3_DC);
}
void servos_init_pos()
{
  PWM_Instance1->setPWM(Servo1_PIN, PWM_frequency, (70/100.0)*(MAXSERVO1-MINSERVO1)+MINSERVO1);
  PWM_Instance2->setPWM(Servo2_PIN, PWM_frequency, MINSERVO2);
  PWM_Instance3->setPWM(Servo3_PIN, PWM_frequency, MINSERVO3);
  PWM_Instance4->setPWM(Servo4_PIN, PWM_frequency, MAXSERVO4);
}
void servos_sort_pos_4rot()
{
  PWM_Instance1->setPWM(Servo1_PIN, PWM_frequency, 11.3);
  PWM_Instance2->setPWM(Servo2_PIN, PWM_frequency, MINSERVO2);
  PWM_Instance3->setPWM(Servo3_PIN, PWM_frequency, MINSERVO3);
  PWM_Instance4->setPWM(Servo4_PIN, PWM_frequency, MINSERVO4);
}
void servos_sort_pos_4dist()
{
  PWM_Instance1->setPWM(Servo1_PIN, PWM_frequency, 11.3);
  PWM_Instance2->setPWM(Servo2_PIN, PWM_frequency, 10.1);
  PWM_Instance3->setPWM(Servo3_PIN, PWM_frequency, 10.8);
  PWM_Instance4->setPWM(Servo4_PIN, PWM_frequency, MINSERVO4);
}
void writeflash()
{
  EEPROM.write(0, PRED_POS.rot);
  EEPROM.write(1, PRED_POS.dist);
  EEPROM.write(2, GRID[0].rot);
  EEPROM.write(3, GRID[0].dist);
  EEPROM.write(4, GRID[1].rot);
  EEPROM.write(5, GRID[1].dist);
  EEPROM.write(6, GRID[2].rot);
  EEPROM.write(7, GRID[2].dist);
  EEPROM.write(8, GRID[3].rot);
  EEPROM.write(9, GRID[3].dist);
  EEPROM.write(10, GRID[4].rot);
  EEPROM.write(11, GRID[4].dist);
  EEPROM.write(12, GRID[5].rot);
  EEPROM.write(13, GRID[5].dist);
  EEPROM.write(14, GRID[6].rot);
  EEPROM.write(15, GRID[6].dist);
  EEPROM.write(16, GRID[7].rot);
  EEPROM.write(17, GRID[7].dist);
  EEPROM.write(18, GRID[8].rot);
  EEPROM.write(19, GRID[8].dist);
  EEPROM.write(20, SORT_RED_POS.rot);
  EEPROM.write(21, SORT_RED_POS.dist);
  EEPROM.write(22, SORT_GREEN_POS.rot);
  EEPROM.write(23, SORT_GREEN_POS.dist);
  EEPROM.write(24, SORT_BLUE_POS.rot);
  EEPROM.write(25, SORT_BLUE_POS.dist);
  EEPROM.write(26, SORT_YELLOW_POS.rot);
  EEPROM.write(27, SORT_YELLOW_POS.dist);
  EEPROM.write(28, SORT_MAGENTA_POS.rot);
  EEPROM.write(29, SORT_MAGENTA_POS.dist);
  EEPROM.write(30, SORT_CYAN_POS.rot);
  EEPROM.write(31, SORT_CYAN_POS.dist);
  EEPROM.commit();
  Serial.print("Data Saved\n");
}
void readflash(){
  PRED_POS.rot = EEPROM.read(0);
  PRED_POS.dist = EEPROM.read(1);
  GRID[0].rot = EEPROM.read(2);
  GRID[0].dist = EEPROM.read(3);
  GRID[1].rot = EEPROM.read(4);
  GRID[1].dist = EEPROM.read(5);
  GRID[2].rot = EEPROM.read(6);
  GRID[2].dist = EEPROM.read(7);
  GRID[3].rot = EEPROM.read(8);
  GRID[3].dist = EEPROM.read(9);
  GRID[4].rot = EEPROM.read(10);
  GRID[4].dist = EEPROM.read(11);
  GRID[5].rot = EEPROM.read(12);
  GRID[5].dist = EEPROM.read(13);
  GRID[6].rot = EEPROM.read(14);
  GRID[6].dist = EEPROM.read(15);
  GRID[7].rot = EEPROM.read(16);
  GRID[7].dist = EEPROM.read(17);
  GRID[8].rot = EEPROM.read(18);
  GRID[8].dist = EEPROM.read(19);
  SORT_RED_POS.rot = EEPROM.read(20);
  SORT_RED_POS.dist = EEPROM.read(21);
  SORT_GREEN_POS.rot = EEPROM.read(22);
  SORT_GREEN_POS.dist = EEPROM.read(23);
  SORT_BLUE_POS.rot = EEPROM.read(24);
  SORT_BLUE_POS.dist = EEPROM.read(25);
  SORT_YELLOW_POS.rot = EEPROM.read(26);
  SORT_YELLOW_POS.dist = EEPROM.read(27);
  SORT_MAGENTA_POS.rot = EEPROM.read(28);
  SORT_MAGENTA_POS.dist = EEPROM.read(29);
  SORT_CYAN_POS.rot = EEPROM.read(30);
  SORT_CYAN_POS.dist = EEPROM.read(31);
  Serial.print("Data Backed Up\n");
}

void setup(){
  Serial.begin(115200);
  Serial.print("Starting...\n");

  EEPROM.begin(EEPROM.length());

  PWM_frequency = 50;
  PWM_Instance1 = new RP2040_PWM(Servo1_PIN, PWM_frequency, Servo1_DC);
  PWM_Instance2 = new RP2040_PWM(Servo2_PIN, PWM_frequency, Servo2_DC);
  PWM_Instance3 = new RP2040_PWM(Servo3_PIN, PWM_frequency, Servo3_DC);
  PWM_Instance4 = new RP2040_PWM(Servo4_PIN, PWM_frequency, Servo4_DC);
                    
  Wire.setSDA(16);
  Wire.setSCL(17);

  Serial.print("Starting I2C...\n");
  

  Wire1.setSDA(26);
  Wire1.setSCL(27);

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

    if (tof.readRangeAvailable()) 
    {
    prev_distance = distance;
    distance = tof.readRangeMillimeters() * 1e-3;
    }
    tof.startReadRangeMillimeters();

    uint8_t key;
    if (Serial.available()) {
      
      key = Serial.read();  

      if(key == 'z'){
        MAIN_MODE=1;
      }
      else if(key == 'x'){
        MAIN_MODE=2;
      }
      else if(key == 'c'){
        MAIN_MODE=3;
      }
      else if(key == 'v')
      {
        MAIN_MODE=4;
      }
      else if(key == 'p'){
        count_print=100;
      }

      if(OP_MAIN_MODE==1)
      {

      }
      else if(OP_MAIN_MODE==2)
      {
        if(key == 's'){
          START=1;
        }
        else if(key == 'q'){
          MODE=1;
        }
        else if(key == 'e'){
          MODE=3;
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
      }
      else if(OP_MAIN_MODE==3)
      {
        if(key == 'n'){
          CONFIG_SWITCH=1;
        }
        else if(key == 'm'){
          CONFIG_TYPES_SWITCH=1;
        }
        else if(key == 'w'){
          if(cs_config != WAIT_CONFIG){
            config_dist++;
          }
        }
        else if(key == 's'){
          if(cs_config != WAIT_CONFIG){
            config_dist--;
          }
        }
        else if(key == 'a'){
          if(cs_config != WAIT_CONFIG){
            config_rot--;
          }
        }
        else if(key == 'd'){
          if(cs_config != WAIT_CONFIG){
            config_rot++;
          }
        }
        else if(key == 'q'){
          SAVE_CONFIG=1;
          }
        }
      else if(OP_MAIN_MODE==4)
      {
        if(key == 's')
        {
          writeflash();
        }
        else if(key == 'b')
        {
          readflash();
        }
      }

      if(config_dist<0) config_dist=0;
      if(config_rot<0) config_rot=0;

      if(config_dist>100) config_dist=100;
      if(config_rot>100) config_rot=100;
      }

    uint16_t r, g, b, c, colorTemp, lux;
    getRawData_noDelay(&r, &g, &b, &c);
    colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);

    if(count_print==100)
    {
      for(int i=0;i<20;i++)
      {
        Serial.print("\n");
      }
      count_print=0;
      if(OP_MAIN_MODE==1)
      {
        Serial.print("StandBy Mode\n");
      }
      else if(OP_MAIN_MODE==2)
      {
        Serial.print("Operation Mode\n");
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

        Serial.print("Distance: ");
        Serial.print(distance);
        Serial.print("\n");
        
        Serial.print("Color : ");
        Serial.print(closestColor(r,g,b));
        Serial.print("\n");

        Serial.print("'S' to start\n");
        Serial.print("'P' to print\n");
        Serial.print("'Q' to change to Predefined Mode\n");
        Serial.print("'E' to change to Sweep Mode\n");
        Serial.print("'1-9' to change to Grid 1-9\n");
      }
      else if(OP_MAIN_MODE==3)
      {
        Serial.print("Config Mode\n");
        Serial.print("Selected Config: ");
        if(cs_config==PRED_POS_CONFIG)
        {
          Serial.print("PRED_POS_CONFIG\n");
        }
        else if(cs_config==GRID_POS_CONFIG)
        {
          Serial.print("GRID_POS_CONFIG\n");
        }
        else if(cs_config==SORT_POS_CONFIG)
        {
          Serial.print("SORT_POS_CONFIG\n");
        }
        else if(cs_config==WAIT_CONFIG)
        {
          Serial.print("WAIT_CONFIG\n");
        }
        Serial.print("'N' to change between configs\n");
        Serial.print("'M' to change between config types\n");
        Serial.print("'W' to increase distance\n");
        Serial.print("'S' to decrease distance\n");
        Serial.print("'A' to decrease rotation\n");
        Serial.print("'D' to increase rotation\n");
        Serial.print("'Q' to save config\n");
        Serial.print("Temp Config Dist: ");
        Serial.print(config_dist);
        Serial.print("\n");
        Serial.print("Temp Config Rot: ");
        Serial.print(config_rot);
        Serial.print("\n");
      }
       
      Serial.print("'Z' to change to StandBy Mode\n");
      Serial.print("'X' to change to Operation Mode\n");
      Serial.print("'C' to change to Config Mode\n");
    }
    if(cs_robotic_arm==WAIT && (cs_move==WAIT_MOVE || cs_move==FINISH) )
    {
      OP_MAIN_MODE=MAIN_MODE;
    }
    
    //STATE MACHINES TRANSITIONS
    switch (cs_main)
    {
    case STANDBY:
      if(OP_MAIN_MODE==2)
      {
        cs_main=OPERATION;
      }
      else if(OP_MAIN_MODE==3)
      {
        cs_main=CONFIG;
      }
      else if(OP_MAIN_MODE==4)
      {
        cs_main=FLASH;
      }
      break;
    case OPERATION:
      if(OP_MAIN_MODE==1)
      {
        cs_main=STANDBY;
      }
      else if(OP_MAIN_MODE==3)
      {
        cs_main=CONFIG;
      }
      else if(OP_MAIN_MODE==4)
      {
        cs_main=FLASH;
      }
      break;
    case CONFIG:
      if(OP_MAIN_MODE==1)
      {
        cs_main=STANDBY;
      }
      else if(OP_MAIN_MODE==2)
      {
        cs_main=OPERATION;
      }
      else if(OP_MAIN_MODE==4)
      {
        cs_main=FLASH;
      }
      break;
    case FLASH:
      if(OP_MAIN_MODE==1)
      {
        cs_main=STANDBY;
      }
      else if(OP_MAIN_MODE==2)
      {
        cs_main=OPERATION;
      }
      else if(OP_MAIN_MODE==3)
      {
        cs_main=CONFIG;
      }
      break;
    }
    switch (cs_robotic_arm)
    {
    case WAIT:
      if(START==1 && cs_main==OPERATION)
      {
        START=0;
        START_MOVE=1;
        lastcs_robotic_arm=SORT;
        cs_robotic_arm=PICK_UP;
        FINISHED=0;
        CLAW_CHANGE=1;
        OP_MODE=MODE;
        CLAW_OPEN_OR_CLOSE=1;

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
      if(SWEEP_FAILED==1)
      {
        cs_robotic_arm=WAIT;
        FINISHED=0;
        SWEEP_FAILED=0;
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
        sort_test_aux=0;
      }
      if(cs_move == SERVOS_ADVANCE && now-SERVOS_ADVANCE_TIMER>800 && sort_test_aux==0)
      {
        sort_test_aux=1;
        if(closestColor(r,g,b)=="red")
        {
          Serial.print("RED DETECTED\n");
          COLOUR_READ=1;
        }
        else if(closestColor(r,g,b)=="green")
        {
          Serial.print("GREEN DETECTED\n");
          COLOUR_READ=2;
        }
        else if(closestColor(r,g,b)=="blue")
        {
          Serial.print("BLUE DETECTED\n");
          COLOUR_READ=3;
        }
        else if(closestColor(r,g,b)=="yellow")
        {
          Serial.print("YELLOW DETECTED\n");
          COLOUR_READ=4;
        }
        else if(closestColor(r,g,b)=="magenta")
        {
          Serial.print("MAGENTA DETECTED\n");
          COLOUR_READ=5;
        }
        else if(closestColor(r,g,b)=="cyan")
        {
          Serial.print("CYAN DETECTED\n");
          COLOUR_READ=6;
        }
        else
        {
          COLOUR_READ=0;
        }
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
          lastcs_robotic_arm=cs_robotic_arm;
          SWEEP_FINISHED=0;
          cs_move=SWEEP_WAIT;
          START_MOVE=0;
          SWEEP_TIMER=now;
        }
        else if(cs_robotic_arm==PICK_UP && OP_MODE==2)
        {
          lastcs_robotic_arm=cs_robotic_arm;
          GRID_FINISHED=0;
          cs_move=WAITING_GRID_CLEAR;
          START_MOVE=0;
          GRID_CLEAR_TIMER=now;
          SERVOS_ROTATE_TIMER=now;
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
    case WAITING_GRID_CLEAR:
      if(GRID_FINISHED==1)
      {
        cs_move=SERVOS_ROTATE;
        SERVOS_ROTATE_TIMER=now;
        GRID_FINISHED=0;
      }
      break;
    case SWEEP_WAIT:
      if(now-SWEEP_TIMER>=1000)
      {
        cs_move=SWEEP;
        SWEEP_COUNT_AUX=30;
      }
      break;
    case SWEEP:
      if(SWEEP_FINISHED==1 && SWEEP_FAILED==0)
      {
        cs_move=SWEEP_RETRACT;
        CLAW_CHANGE=1;
        SWEEP_TIMER=now;
        SWEEP_FINISHED=0;
      }
      else if(SWEEP_FINISHED==1 && SWEEP_FAILED==1)
      {
        cs_move=FINISH;
        FINISHED=1;
      }
      break;
    case SWEEP_RETRACT:
      if(now-SWEEP_TIMER>=WAITING_TIME)
      {
        cs_move=SERVOS_ROTATE;
        SERVOS_ROTATE_TIMER=now;
      }
      break;
    case SERVOS_ROTATE:
      if(now-SERVOS_ROTATE_TIMER>=WAITING_TIME)
      {
        cs_move=SERVOS_ADVANCE;
        SERVOS_ADVANCE_TIMER=now;
      }
      break;
    case SERVOS_ADVANCE:
      if(now-SERVOS_ADVANCE_TIMER>=WAITING_TIME && cs_robotic_arm!=READ_COLOUR)
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
      else if(now-SERVOS_ADVANCE_TIMER>=1000 && cs_robotic_arm==READ_COLOUR)
      {
        cs_move=RETRACT;
        RETRACT_TIMER=now;
      }
      break;
    case CLAW:
      if(now-CLAW_TIMER>=0.5*WAITING_TIME)
      {
        RETRACT_TIMER=now;
        cs_move=RETRACT;
      }
      break;
    case RETRACT:
      if(now-RETRACT_TIMER>=WAITING_TIME)
      {
        if(cs_robotic_arm==SORT && OP_MODE==2)
        {
          cs_move=WAITING_GRID_RESTORE;
          GRID_CLEAR_TIMER=now;
        }
        else
        {
          cs_move=FINISH;
          FINISHED=1;
        }
      }
      break;
    case WAITING_GRID_RESTORE:
      if(GRID_RESTORE_FINISHED==1)
      {
        cs_move=FINISH;
        GRID_RESTORE_FINISHED=0;
        FINISHED=1;
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
    switch (cs_grid_clear)
    {
    case WAIT_GRID:
      if(cs_move==WAITING_GRID_CLEAR)
      {
        if(OP_GOTOGRID==1 || OP_GOTOGRID==4 || OP_GOTOGRID==7)
        {
          GRID_FINISHED=1;
        }
        else
        {
          cs_grid_clear=ROT_147;
          GRID_CLEAR_TIMER=now;
        }
      }
      break;
    case ROT_147:
      if(now-GRID_CLEAR_TIMER>=WAITING_TIME)
      {
        cs_grid_clear=ADV_147;
        GRID_CLEAR_TIMER=now;
      }
      break;
    case ADV_147:
      if(now-GRID_CLEAR_TIMER>=WAITING_TIME)
      {
        cs_grid_clear=PICK_147;
        GRID_CLEAR_TIMER=now;
      }
      break;
    case PICK_147:
      if(now-GRID_CLEAR_TIMER>=WAITING_TIME)
      {
        cs_grid_clear=RETRACT_147;
        GRID_CLEAR_TIMER=now;
      }
      break;
    case RETRACT_147:
      if(now-GRID_CLEAR_TIMER>=WAITING_TIME)
      {
        cs_grid_clear=ROT_RETRACT_147;
        GRID_CLEAR_TIMER=now;
      }
      break;
    case ROT_RETRACT_147:
      if(now-GRID_CLEAR_TIMER>=WAITING_TIME)
      {
        cs_grid_clear=PUT_147;
        GRID_CLEAR_TIMER=now;
      }
      break;
    case PUT_147:
      if(now-GRID_CLEAR_TIMER>=WAITING_TIME)
      {
        cs_grid_clear=OPEN_CLAW_147;
        GRID_CLEAR_TIMER=now;
      }
      break;
    case OPEN_CLAW_147:
      if(now-GRID_CLEAR_TIMER>=WAITING_TIME)
      {
        cs_grid_clear=PUT_RETRACT_147;
        GRID_CLEAR_TIMER=now;
      }
      break;
    case PUT_RETRACT_147:
      if(now-GRID_CLEAR_TIMER>=WAITING_TIME)
      {
        cs_grid_clear=ROT_258;
        GRID_CLEAR_TIMER=now;
      }
      break;
    case ROT_258:
      if(OP_GOTOGRID==2 || OP_GOTOGRID==5 || OP_GOTOGRID==8)
      {
        GRID_FINISHED=1;
      }
      else if(now-GRID_CLEAR_TIMER>=WAITING_TIME)
      {
        cs_grid_clear=ADV_258;
        GRID_CLEAR_TIMER=now;
      }
      break;
    case ADV_258:
      if(now-GRID_CLEAR_TIMER>=WAITING_TIME)
      {
        cs_grid_clear=PICK_258;
        GRID_CLEAR_TIMER=now;
      }
      break;
    case PICK_258:
      if(now-GRID_CLEAR_TIMER>=WAITING_TIME)
      {
        cs_grid_clear=RETRACT_258;
        GRID_CLEAR_TIMER=now;
      }
      break;
    case RETRACT_258:
      if(now-GRID_CLEAR_TIMER>=WAITING_TIME)
      {
        cs_grid_clear=ROT_RETRACT_258;
        GRID_CLEAR_TIMER=now;
      }
      break;
    case ROT_RETRACT_258:
      if(now-GRID_CLEAR_TIMER>=WAITING_TIME)
      {
        cs_grid_clear=PUT_258;
        GRID_CLEAR_TIMER=now;
      }
      break;
    case PUT_258:
      if(now-GRID_CLEAR_TIMER>=WAITING_TIME)
      {
        cs_grid_clear=OPEN_CLAW_258;
        GRID_CLEAR_TIMER=now;
      }
      break;
    case OPEN_CLAW_258: 
      if(now-GRID_CLEAR_TIMER>=WAITING_TIME)
      {
        cs_grid_clear=PUT_RETRACT_258;
        GRID_CLEAR_TIMER=now;
      }
      break;
    case PUT_RETRACT_258:
      if(now-GRID_CLEAR_TIMER>=WAITING_TIME)
      {
        cs_grid_clear=WAIT_GRID;
        GRID_CLEAR_TIMER=now;
        GRID_FINISHED=1;
      }
      break;
    default:
      break;
    }
    switch (cs_grid_restore)
    { 
    case WAIT_GRID_RESTORE:
      if(cs_move==WAITING_GRID_RESTORE)
      {
        if(OP_GOTOGRID==1 || OP_GOTOGRID==4 || OP_GOTOGRID==7)
        {
          GRID_RESTORE_FINISHED=1;
        }
        else if(OP_GOTOGRID==2 || OP_GOTOGRID==5 || OP_GOTOGRID==8)
        {
          cs_grid_restore=ROT_147_RESTORE;
          GRID_CLEAR_TIMER=now;
        }
        else if(OP_GOTOGRID==3 || OP_GOTOGRID==6 || OP_GOTOGRID==9)
        {
          cs_grid_restore=ROT_258_RESTORE;
          GRID_CLEAR_TIMER=now;
        }
      }
      break;
    case ROT_258_RESTORE:
      if(now-GRID_CLEAR_TIMER>=WAITING_TIME)
      {
        cs_grid_restore=ADV_258_RESTORE;
        GRID_CLEAR_TIMER=now;
      }
      break;
    case ADV_258_RESTORE:
      if(now-GRID_CLEAR_TIMER>=WAITING_TIME)
      {
        cs_grid_restore=PICK_258_RESTORE;
        GRID_CLEAR_TIMER=now;
      }
      break;
    case PICK_258_RESTORE:
      if(now-GRID_CLEAR_TIMER>=WAITING_TIME)
      {
        cs_grid_restore=RETRACT_258_RESTORE;
        GRID_CLEAR_TIMER=now;
      }
      break;
    case RETRACT_258_RESTORE:
      if(now-GRID_CLEAR_TIMER>=WAITING_TIME)
      {
        cs_grid_restore=ROT_RETRACT_258_RESTORE;
        GRID_CLEAR_TIMER=now;
      }
      break;
    case ROT_RETRACT_258_RESTORE:
      if(now-GRID_CLEAR_TIMER>=WAITING_TIME)
      {
        cs_grid_restore=PUT_258_RESTORE;
        GRID_CLEAR_TIMER=now;
      }
      break;
    case PUT_258_RESTORE:
      if(now-GRID_CLEAR_TIMER>=WAITING_TIME)
      {
        cs_grid_restore=OPEN_CLAW_258_RESTORE;
        GRID_CLEAR_TIMER=now;
      }
      break;
    case OPEN_CLAW_258_RESTORE:
      if(now-GRID_CLEAR_TIMER>=WAITING_TIME)
      {
        cs_grid_restore=PUT_RETRACT_258_RESTORE;
        GRID_CLEAR_TIMER=now;
      }
      break;
    case PUT_RETRACT_258_RESTORE:
      if(now-GRID_CLEAR_TIMER>=WAITING_TIME)
      {
        cs_grid_restore=ROT_147_RESTORE;
        GRID_CLEAR_TIMER=now;
      }
      break;
    case ROT_147_RESTORE:
      if(now-GRID_CLEAR_TIMER>=WAITING_TIME)
      {
        cs_grid_restore=ADV_147_RESTORE;
        GRID_CLEAR_TIMER=now;
      }
      break;
    case ADV_147_RESTORE:
      if(now-GRID_CLEAR_TIMER>=WAITING_TIME)
      {
        cs_grid_restore=PICK_147_RESTORE;
        GRID_CLEAR_TIMER=now;
      }
      break;
    case PICK_147_RESTORE:
      if(now-GRID_CLEAR_TIMER>=WAITING_TIME)
      {
        cs_grid_restore=RETRACT_147_RESTORE;
        GRID_CLEAR_TIMER=now;
      }
      break;
    case RETRACT_147_RESTORE:
      if(now-GRID_CLEAR_TIMER>=WAITING_TIME)
      {
        cs_grid_restore=ROT_RETRACT_147_RESTORE;
        GRID_CLEAR_TIMER=now;
      }
      break;
    case ROT_RETRACT_147_RESTORE:
      if(now-GRID_CLEAR_TIMER>=WAITING_TIME)
      {
        cs_grid_restore=PUT_147_RESTORE;
        GRID_CLEAR_TIMER=now;
      }
      break;
    case PUT_147_RESTORE:
      if(now-GRID_CLEAR_TIMER>=WAITING_TIME)
      {
        cs_grid_restore=OPEN_CLAW_147_RESTORE;
        GRID_CLEAR_TIMER=now;
      }
      break;
    case OPEN_CLAW_147_RESTORE:
      if(now-GRID_CLEAR_TIMER>=WAITING_TIME)
      {
        cs_grid_restore=PUT_RETRACT_147_RESTORE;
        GRID_CLEAR_TIMER=now;
      }
      break;
    case PUT_RETRACT_147_RESTORE:
      if(now-GRID_CLEAR_TIMER>=WAITING_TIME)
      {
        cs_grid_restore=WAIT_GRID_RESTORE;
        GRID_CLEAR_TIMER=now;
        GRID_RESTORE_FINISHED=1;
      }
      break;
    default:
      break;
    }
    switch (cs_config)
    {
    case WAIT_CONFIG:
      if(CONFIG_SWITCH==1 && cs_main==CONFIG)
      {
        CONFIG_SWITCH=0;
        cs_config=PRED_POS_CONFIG;
        config_dist=PRED_POS.dist;
        config_rot=PRED_POS.rot;
      }
      break;
    case PRED_POS_CONFIG:
      if(CONFIG_SWITCH==1 && cs_main==CONFIG)
      {
        CONFIG_SWITCH=0;
        cs_config=GRID_POS_CONFIG;
        config_dist=GRID[0].dist;
        config_rot=GRID[0].rot;
      }
      else if(SAVE_CONFIG==1)
      {
        Serial.print("PRED_POS Config Saved\n");
        PRED_POS.dist=config_dist;
        PRED_POS.rot=config_rot;
        SAVE_CONFIG=0;
      }
      break;
    case GRID_POS_CONFIG:
      if(CONFIG_SWITCH==1 && cs_main==CONFIG)
      {
        CONFIG_SWITCH=0;
        cs_config=SORT_POS_CONFIG;
        config_dist=SORT_GREEN_POS.dist;
        config_rot=SORT_GREEN_POS.rot;
      }
      break;
    case SORT_POS_CONFIG:
      if(CONFIG_SWITCH==1 && cs_main==CONFIG)
      {
        CONFIG_SWITCH=0;
        cs_config=WAIT_CONFIG;
        config_dist=50;
        config_rot=50;
      }
      break;
    
    default:
      break;
    }
    switch (cs_grid_type)
    {
    case WAIT_GRID_TYPE:
      if(cs_config==GRID_POS_CONFIG)
      {
        Serial.print("GRID 1 Config\n");
        config_dist=GRID[0].dist;
        config_rot=GRID[0].rot;
        CONFIG_TYPES_SWITCH=0;
        cs_grid_type=GRID1;
      }
      break;
    case GRID1:
      if(CONFIG_TYPES_SWITCH==1 && cs_config==GRID_POS_CONFIG)
      {
        Serial.print("GRID 2 Config\n");
        config_dist=GRID[1].dist;
        config_rot=GRID[1].rot;
        CONFIG_TYPES_SWITCH=0;
        cs_grid_type=GRID2;
      }
      else if(cs_config!=GRID_POS_CONFIG)
      {
        cs_grid_type=WAIT_GRID_TYPE;
      }
      else if(SAVE_CONFIG==1)
      {
        GRID[0].dist=config_dist;
        GRID[0].rot=config_rot;
        SAVE_CONFIG=0;
      }
      break;
    case GRID2:
      if(CONFIG_TYPES_SWITCH==1 && cs_config==GRID_POS_CONFIG)
      {
        Serial.print("GRID 3 Config\n");
        config_dist=GRID[2].dist;
        config_rot=GRID[2].rot;
        CONFIG_TYPES_SWITCH=0;
        cs_grid_type=GRID3;
      }
      else if(cs_config!=GRID_POS_CONFIG)
      {
        cs_grid_type=WAIT_GRID_TYPE;
      }
      else if(SAVE_CONFIG==1)
      {
        GRID[1].dist=config_dist;
        GRID[1].rot=config_rot;
        SAVE_CONFIG=0;
      }
      break;
    case GRID3:
      if(CONFIG_TYPES_SWITCH==1 && cs_config==GRID_POS_CONFIG)
      {
        Serial.print("GRID 4 Config\n");
        config_dist=GRID[3].dist;
        config_rot=GRID[3].rot;
        CONFIG_TYPES_SWITCH=0;
        cs_grid_type=GRID4;
      }
      else if(cs_config!=GRID_POS_CONFIG)
      {
        cs_grid_type=WAIT_GRID_TYPE;
      }
      else if(SAVE_CONFIG==1)
      {
        GRID[2].dist=config_dist;
        GRID[2].rot=config_rot;
        SAVE_CONFIG=0;
      }
      break;
    case GRID4:
      if(CONFIG_TYPES_SWITCH==1 && cs_config==GRID_POS_CONFIG)
      {
        Serial.print("GRID 5 Config\n");
        config_dist=GRID[4].dist;
        config_rot=GRID[4].rot;
        CONFIG_TYPES_SWITCH=0;
        cs_grid_type=GRID5;
      }
      else if(cs_config!=GRID_POS_CONFIG)
      {
        cs_grid_type=WAIT_GRID_TYPE;
      }
      else if(SAVE_CONFIG==1)
      {
        GRID[3].dist=config_dist;
        GRID[3].rot=config_rot;
        SAVE_CONFIG=0;
      }
      break;
    case GRID5:
      if(CONFIG_TYPES_SWITCH==1 && cs_config==GRID_POS_CONFIG)
      {
        Serial.print("GRID 6 Config\n");
        config_dist=GRID[5].dist;
        config_rot=GRID[5].rot;
        CONFIG_TYPES_SWITCH=0;
        cs_grid_type=GRID6;
      }
      else if(cs_config!=GRID_POS_CONFIG)
      {
        cs_grid_type=WAIT_GRID_TYPE;
      }
      else if(SAVE_CONFIG==1)
      {
        GRID[4].dist=config_dist;
        GRID[4].rot=config_rot;
        SAVE_CONFIG=0;
      }
      break;
    case GRID6:
      if(CONFIG_TYPES_SWITCH==1 && cs_config==GRID_POS_CONFIG)
      {
        Serial.print("GRID 7 Config\n");
        config_dist=GRID[6].dist;
        config_rot=GRID[6].rot;
        CONFIG_TYPES_SWITCH=0;
        cs_grid_type=GRID7;
      }
      else if(cs_config!=GRID_POS_CONFIG)
      {
        cs_grid_type=WAIT_GRID_TYPE;
      }
      else if(SAVE_CONFIG==1)
      {
        GRID[5].dist=config_dist;
        GRID[5].rot=config_rot;
        SAVE_CONFIG=0;
      }
      break;
    case GRID7:
      if(CONFIG_TYPES_SWITCH==1 && cs_config==GRID_POS_CONFIG)
      {
        Serial.print("GRID 8 Config\n");
        config_dist=GRID[7].dist;
        config_rot=GRID[7].rot;
        CONFIG_TYPES_SWITCH=0;
        cs_grid_type=GRID8;
      }
      else if(cs_config!=GRID_POS_CONFIG)
      {
        cs_grid_type=WAIT_GRID_TYPE;
      }
      else if(SAVE_CONFIG==1)
      {
        GRID[6].dist=config_dist;
        GRID[6].rot=config_rot;
        SAVE_CONFIG=0;
      }
      break;
    case GRID8:
      if(CONFIG_TYPES_SWITCH==1 && cs_config==GRID_POS_CONFIG)
      {
        Serial.print("GRID 9 Config\n");
        config_dist=GRID[8].dist;
        config_rot=GRID[8].rot;
        CONFIG_TYPES_SWITCH=0;
        cs_grid_type=GRID9;
      }
      else if(cs_config!=GRID_POS_CONFIG)
      {
        cs_grid_type=WAIT_GRID_TYPE;
      }
      else if(SAVE_CONFIG==1)
      {
        GRID[7].dist=config_dist;
        GRID[7].rot=config_rot;
        SAVE_CONFIG=0;
      }
      break;
    case GRID9:
      if(CONFIG_TYPES_SWITCH==1 && cs_config==GRID_POS_CONFIG)
      {
        Serial.print("GRID 1 Config\n");
        config_dist=GRID[0].dist;
        config_rot=GRID[0].rot;
        CONFIG_TYPES_SWITCH=0;
        cs_grid_type=GRID1;
      }
      else if(cs_config!=GRID_POS_CONFIG)
      {
        cs_grid_type=WAIT_GRID_TYPE;
      }
      else if(SAVE_CONFIG==1)
      {
        GRID[8].dist=config_dist;
        GRID[8].rot=config_rot;
        SAVE_CONFIG=0;
      }
      break;

    default:
      break;

    }
    switch (cs_colour_type)
    {
    case WAIT_COLOUR_TYPE:
      if(cs_config==SORT_POS_CONFIG)
      {
        Serial.print("Red Config\n");
        CONFIG_TYPES_SWITCH=0;
        cs_colour_type=RED;
      }
      break;
    case RED:
      if(CONFIG_TYPES_SWITCH==1 && cs_config==SORT_POS_CONFIG)
      {
        Serial.print("Green Config\n");
        config_dist=SORT_GREEN_POS.dist;
        config_rot=SORT_GREEN_POS.rot;
        CONFIG_TYPES_SWITCH=0;
        cs_colour_type=GREEN;
      }
      else if(cs_config!=SORT_POS_CONFIG)
      {
        cs_colour_type=WAIT_COLOUR_TYPE;
      }
      else if(SAVE_CONFIG==1)
      {
        SORT_GREEN_POS.dist=config_dist;
        SORT_GREEN_POS.rot=config_rot;
        SAVE_CONFIG=0;
      }
      break;
    case GREEN:
      if(CONFIG_TYPES_SWITCH==1 && cs_config==SORT_POS_CONFIG)
      {
        Serial.print("Blue Config\n");
        config_dist=SORT_BLUE_POS.dist;
        config_rot=SORT_BLUE_POS.rot;
        CONFIG_TYPES_SWITCH=0;
        cs_colour_type=BLUE;
      }
      else if(cs_config!=SORT_POS_CONFIG)
      {
        cs_colour_type=WAIT_COLOUR_TYPE;
      }
      else if(SAVE_CONFIG==1)
      {
        SORT_BLUE_POS.dist=config_dist;
        SORT_BLUE_POS.rot=config_rot;
        SAVE_CONFIG=0;
      }
      break;
    case BLUE:
      if(CONFIG_TYPES_SWITCH==1 && cs_config==SORT_POS_CONFIG)
      {
        Serial.print("Yellow Config\n");
        config_dist=SORT_YELLOW_POS.dist;
        config_rot=SORT_YELLOW_POS.rot;
        CONFIG_TYPES_SWITCH=0;
        cs_colour_type=RED;
      }
      else if(cs_config!=SORT_POS_CONFIG)
      {
        cs_colour_type=WAIT_COLOUR_TYPE;
      }
      else if(SAVE_CONFIG==1)
      {
        SORT_YELLOW_POS.dist=config_dist;
        SORT_YELLOW_POS.rot=config_rot;
        SAVE_CONFIG=0;
      }
      break;
    case YELLOW:
      if(CONFIG_TYPES_SWITCH==1 && cs_config==SORT_POS_CONFIG)
      {
        Serial.print("Magenta Config\n");
        config_dist=SORT_MAGENTA_POS.dist;
        config_rot=SORT_MAGENTA_POS.rot;
        CONFIG_TYPES_SWITCH=0;
        cs_colour_type=MAGENTA;
      }
      else if(cs_config!=SORT_POS_CONFIG)
      {
        cs_colour_type=WAIT_COLOUR_TYPE;
      }
      else if(SAVE_CONFIG==1)
      {
        SORT_MAGENTA_POS.dist=config_dist;
        SORT_MAGENTA_POS.rot=config_rot;
        SAVE_CONFIG=0;
      }
      break;
    case MAGENTA:
      if(CONFIG_TYPES_SWITCH==1 && cs_config==SORT_POS_CONFIG)
      {
        Serial.print("Cyan Config\n");
        config_dist=SORT_CYAN_POS.dist;
        config_rot=SORT_CYAN_POS.rot;
        CONFIG_TYPES_SWITCH=0;
        cs_colour_type=CYAN;
      }
      else if(cs_config!=SORT_POS_CONFIG)
      {
        cs_colour_type=WAIT_COLOUR_TYPE;
      }
      else if(SAVE_CONFIG==1)
      {
        SORT_CYAN_POS.dist=config_dist;
        SORT_CYAN_POS.rot=config_rot;
        SAVE_CONFIG=0;
      }
      break;
    case CYAN:
      if(CONFIG_TYPES_SWITCH==1 && cs_config==SORT_POS_CONFIG)
      {
        Serial.print("Red Config\n");
        config_dist=SORT_RED_POS.dist;
        config_rot=SORT_RED_POS.rot;
        CONFIG_TYPES_SWITCH=0;
        cs_colour_type=RED;
      }
      else if(cs_config!=SORT_POS_CONFIG)
      {
        cs_colour_type=WAIT_COLOUR_TYPE;
      }
      else if(SAVE_CONFIG==1)
      {
        SORT_RED_POS.dist=config_dist;
        SORT_RED_POS.rot=config_rot;
        SAVE_CONFIG=0;
      }
      break;
    default:
      break;
    }
    //STATE MACHINES ACTIONS
    switch (cs_move)
    {
    case SERVOS_ROTATE:
      if(cs_robotic_arm==PICK_UP && OP_MODE==1)
      {
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
        servos_sort_pos_4rot();
      }
      else if(cs_robotic_arm==SORT)
      {
        if(COLOUR_READ==1)
        {
          servos_write(SORT_RED_POS.rot,-1,-1);
        }
        else if(COLOUR_READ==2)
        {
          servos_write(SORT_GREEN_POS.rot,-1,-1);
        }
        else if(COLOUR_READ==3)
        {
          servos_write(SORT_BLUE_POS.rot,-1,-1);
        }
        else if(COLOUR_READ==4)
        {
          servos_write(SORT_YELLOW_POS.rot,-1,-1);
        }
        else if(COLOUR_READ==5)
        {
          servos_write(SORT_MAGENTA_POS.rot,-1,-1);
        }
        else if(COLOUR_READ==6)
        {
          servos_write(SORT_CYAN_POS.rot,-1,-1);
        }
        else if(COLOUR_READ==0)
        {
          servos_write(SORT_UNDETECTED_POS.rot,-1,-1);
        }
      }
      break;
    case SERVOS_ADVANCE:
      if(cs_robotic_arm==PICK_UP && OP_MODE==1)
      {
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
        servos_sort_pos_4dist();
      }
      else if(cs_robotic_arm==SORT)
      {
        if(COLOUR_READ==1)
        {
          servos_write(SORT_RED_POS.rot,SORT_RED_POS.dist,SORT_RED_POS.dist);
        }
        else if(COLOUR_READ==2)
        {
          servos_write(SORT_GREEN_POS.rot,SORT_GREEN_POS.dist,SORT_GREEN_POS.dist);
        }
        else if(COLOUR_READ==3)
        {
          servos_write(SORT_BLUE_POS.rot,SORT_BLUE_POS.dist,SORT_BLUE_POS.dist);
        }
        else if(COLOUR_READ==4)
        {
          servos_write(SORT_YELLOW_POS.rot,SORT_YELLOW_POS.dist,SORT_YELLOW_POS.dist);
        }
        else if(COLOUR_READ==5)
        {
          servos_write(SORT_MAGENTA_POS.rot,SORT_MAGENTA_POS.dist,SORT_MAGENTA_POS.dist);
        }
        else if(COLOUR_READ==6)
        {
          servos_write(SORT_CYAN_POS.rot,SORT_CYAN_POS.dist,SORT_CYAN_POS.dist);
        }
        else if(COLOUR_READ==0)
        {
          servos_write(SORT_UNDETECTED_POS.rot,SORT_RED_POS.dist,SORT_RED_POS.dist);
        }

      }
      break;
    case RETRACT:
      if(cs_robotic_arm==PICK_UP && OP_MODE==1)
      {
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
        servos_sort_pos_4rot();
      }
      else if(cs_robotic_arm==SORT)
      {
        if(COLOUR_READ==1)
        {
          servos_write(SORT_RED_POS.rot,-1,-1);
        }
        else if(COLOUR_READ==2)
        {
          servos_write(SORT_GREEN_POS.rot,-1,-1);
        }
        else if(COLOUR_READ==3)
        {
          servos_write(SORT_BLUE_POS.rot,-1,-1);
        }
        else if(COLOUR_READ==4)
        {
          servos_write(SORT_YELLOW_POS.rot,-1,-1);
        }
        else if(COLOUR_READ==5)
        {
          servos_write(SORT_MAGENTA_POS.rot,-1,-1);
        }
        else if(COLOUR_READ==6)
        {
          servos_write(SORT_CYAN_POS.rot,-1,-1);
        }
        else if(COLOUR_READ==0)
        {
          servos_write(SORT_UNDETECTED_POS.rot,-1,-1);
        }
      }
      break;
    case CLAW:
      if(CLAW_OPEN_OR_CLOSE==1)
      {
        PWM_Instance4->setPWM(Servo4_PIN, PWM_frequency, MAXSERVO4);
      }
      else
      {
        PWM_Instance4->setPWM(Servo4_PIN, PWM_frequency, MINSERVO4);
      }
      break;
    case SWEEP_WAIT:
      PWM_Instance1->setPWM(Servo1_PIN, PWM_frequency, (30/100.0)*(MAXSERVO1-MINSERVO1)+MINSERVO1);
      PWM_Instance2->setPWM(Servo2_PIN, PWM_frequency, MINSERVO2);
      PWM_Instance3->setPWM(Servo3_PIN, PWM_frequency, MAXSERVO3+2);
      PWM_Instance4->setPWM(Servo4_PIN, PWM_frequency, MINSERVO4);
      break;
    case SWEEP:
      PWM_Instance1->setPWM(Servo1_PIN, PWM_frequency, (SWEEP_COUNT_AUX/100.0)*(MAXSERVO1-MINSERVO1)+MINSERVO1);
      PWM_Instance2->setPWM(Servo2_PIN, PWM_frequency, MINSERVO2);
      PWM_Instance3->setPWM(Servo3_PIN, PWM_frequency, MAXSERVO3+2);
      PWM_Instance4->setPWM(Servo4_PIN, PWM_frequency, MINSERVO4);
        if(SWEEP_COUNT_AUX<100)
        {
          SWEEP_COUNT[SWEEP_COUNT_AUX]=distance;
          SWEEP_COUNT_AUX++;
        }
        else
        {
          int POS_DET_COUNT=0;
          int i_start=0;
          bool close=0;
          for (int i=0;i<SWEEP_COUNT_AUX;i++)
          {
            Serial.print("SWEEP COUNT ");
            Serial.print(i);
            Serial.print(" : ");
            Serial.print(SWEEP_COUNT[i]);
            Serial.print("\n");
            if(SWEEP_COUNT[i]<DIST_MAX && SWEEP_COUNT[i]>DIST_MIN && close==0)
            {
              POS_DET_COUNT++;
              if(i_start==0)
              {
                i_start=i;
              }
              if(i_start!=0 && SWEEP_COUNT[i]<DIST_MAX && SWEEP_COUNT[i]>DIST_MIN)
              {
                close=1; 
              }
            }
          }
          Serial.print("POS DET COUNT : ");
          Serial.print(POS_DET_COUNT);
          Serial.print("\n");
          if(POS_DET_COUNT==0)
          {
            Serial.print("NO POS DETECTED\n");
            SWEEP_FINISHED=1;
            SWEEP_FAILED=1;
          }
          else
          {
          float dist=0;
          int rot=0;
          for(int i=0;i<POS_DET_COUNT;i++)
          {
            dist+=SWEEP_COUNT[i+i_start];
            rot+=i+i_start;
          }
          UNKNOWN_POS.rot=int(rot/POS_DET_COUNT);
          UNKNOWN_POS.dist=int(((dist/POS_DET_COUNT)-DIST_MIN)/(DIST_MAX-DIST_MIN)*100);

          Serial.print("UNKNOWN POS ROT : ");
          Serial.print(UNKNOWN_POS.rot);
          Serial.print(" | UNKNOWN POS DIST : ");
          Serial.print(UNKNOWN_POS.dist);
          Serial.print("\n");
          SWEEP_FINISHED=1;
          }

          for(int i=0;i<SWEEP_COUNT_AUX;i++)
          {
            SWEEP_COUNT[i]=0;
          }

        }
      break;
    case SWEEP_RETRACT:
      servos_write(100,-1,-1);
      PWM_Instance4->setPWM(Servo4_PIN, PWM_frequency, MAXSERVO4);
      break;
    default:
      break;
    }
    switch (cs_grid_clear)
    {
    case ROT_147:
      if(OP_GOTOGRID==2 || OP_GOTOGRID==3)
      {
        servos_write(GRID[1-1].rot,-1,-1);
      }
      else if(OP_GOTOGRID==5 || OP_GOTOGRID==6)
      {
        servos_write(GRID[4-1].rot,-1,-1);
      }
      else if(OP_GOTOGRID==8 || OP_GOTOGRID==9)
      {
        servos_write(GRID[7-1].rot,-1,-1);
      }
      break;
    case ADV_147:
      if(OP_GOTOGRID==2 || OP_GOTOGRID==3)
      {
        servos_write(GRID[1-1].rot,GRID[1-1].dist,GRID[1-1].dist);
      }
      else if(OP_GOTOGRID==5 || OP_GOTOGRID==6)
      {
        servos_write(GRID[4-1].rot,GRID[4-1].dist,GRID[4-1].dist);
      }
      else if(OP_GOTOGRID==8 || OP_GOTOGRID==9)
      {
        servos_write(GRID[7-1].rot,GRID[7-1].dist,GRID[7-1].dist);
      }
      break;
    case PICK_147:
      PWM_Instance4->setPWM(Servo4_PIN, PWM_frequency, MINSERVO4);
      break;
    case RETRACT_147:
      if(OP_GOTOGRID==2 || OP_GOTOGRID==3)
      {
        servos_write(GRID[1-1].rot,-1,-1);
      }
      else if(OP_GOTOGRID==5 || OP_GOTOGRID==6)
      {
        servos_write(GRID[4-1].rot,-1,-1);
      }
      else if(OP_GOTOGRID==8 || OP_GOTOGRID==9)
      {
        servos_write(GRID[7-1].rot,-1,-1);
      }
      break;
    case ROT_RETRACT_147:
      servos_write(0,-1,-1);
      break;
    case PUT_147:
      servos_write(0,100,100);
      break;
    case OPEN_CLAW_147:
      PWM_Instance4->setPWM(Servo4_PIN, PWM_frequency, MAXSERVO4);
      break;
    case PUT_RETRACT_147:
      servos_write(0,-1,-1);
      break;
    case ROT_258:
      if(OP_GOTOGRID==3)
      {
        servos_write(GRID[2-1].rot,-1,-1);
      }
      else if(OP_GOTOGRID==6)
      {
        servos_write(GRID[5-1].rot,-1,-1);
      }
      else if(OP_GOTOGRID==9)
      {
        servos_write(GRID[8-1].rot,-1,-1);
      }
      break;
    case ADV_258:
      if(OP_GOTOGRID==3)
      {
        servos_write(GRID[2-1].rot,GRID[2-1].dist,GRID[2-1].dist);
      }
      else if(OP_GOTOGRID==6)
      {
        servos_write(GRID[5-1].rot,GRID[5-1].dist,GRID[5-1].dist);
      }
      else if(OP_GOTOGRID==9)
      {
        servos_write(GRID[8-1].rot,GRID[8-1].dist,GRID[8-1].dist);
      }
      break;
    case PICK_258:
      PWM_Instance4->setPWM(Servo4_PIN, PWM_frequency, MINSERVO4);
      break;
    case RETRACT_258:
      if(OP_GOTOGRID==3)
      {
        servos_write(GRID[2-1].rot,-1,-1);
      }
      else if(OP_GOTOGRID==6)
      {
        servos_write(GRID[5-1].rot,-1,-1);
      }
      else if(OP_GOTOGRID==9)
      {
        servos_write(GRID[8-1].rot,-1,-1);
      }
      break;
    case ROT_RETRACT_258:
      servos_write(0,-1,-1);
      break;
    case PUT_258:
      servos_write(0,30,30);
      break;
    case OPEN_CLAW_258:
      PWM_Instance4->setPWM(Servo4_PIN, PWM_frequency, MAXSERVO4);
      break;
    case PUT_RETRACT_258:
      servos_write(0,-1,-1);
      break;
    default:
      break;
    }
    switch (cs_grid_restore)
    {
    case ROT_147_RESTORE:
      servos_write(0,-1,-1);
      break;
    case ADV_147_RESTORE:
      servos_write(0,100,100);
      break;
    case PICK_147_RESTORE:
      PWM_Instance4->setPWM(Servo4_PIN, PWM_frequency, MINSERVO4);
      break;
    case RETRACT_147_RESTORE:
      servos_write(0,-1,-1);
      break;
    case ROT_RETRACT_147_RESTORE:
      if(OP_GOTOGRID==2 || OP_GOTOGRID==3)
      {
        servos_write(GRID[1-1].rot,-1,-1);
      }
      else if(OP_GOTOGRID==5 || OP_GOTOGRID==6)
      {
        servos_write(GRID[4-1].rot,-1,-1);
      }
      else if(OP_GOTOGRID==8 || OP_GOTOGRID==9)
      {
        servos_write(GRID[7-1].rot,-1,-1);
      }
      break;
    case PUT_147_RESTORE:
      if(OP_GOTOGRID==2 || OP_GOTOGRID==3)
      {
        servos_write(GRID[1-1].rot,GRID[1-1].dist,GRID[1-1].dist);
      }
      else if(OP_GOTOGRID==5 || OP_GOTOGRID==6)
      {
        servos_write(GRID[4-1].rot,GRID[4-1].dist,GRID[4-1].dist);
      }
      else if(OP_GOTOGRID==8 || OP_GOTOGRID==9)
      {
        servos_write(GRID[7-1].rot,GRID[7-1].dist,GRID[7-1].dist);
      }
      break;
    case OPEN_CLAW_147_RESTORE:
      PWM_Instance4->setPWM(Servo4_PIN, PWM_frequency, MAXSERVO4);
      break;
    case PUT_RETRACT_147_RESTORE:
      if(OP_GOTOGRID==2 || OP_GOTOGRID==3)
      {
        servos_write(GRID[1-1].rot,-1,-1);
      }
      else if(OP_GOTOGRID==5 || OP_GOTOGRID==6)
      {
        servos_write(GRID[4-1].rot,-1,-1);
      }
      else if(OP_GOTOGRID==8 || OP_GOTOGRID==9)
      {
        servos_write(GRID[7-1].rot,-1,-1);
      }
      break;
    case ROT_258_RESTORE:
      servos_write(0,-1,-1);
      break;
    case ADV_258_RESTORE:
      servos_write(0,30,30);
      break;
    case PICK_258_RESTORE:
      PWM_Instance4->setPWM(Servo4_PIN, PWM_frequency, MINSERVO4);
      break;
    case RETRACT_258_RESTORE:
      servos_write(0,-1,-1);
      break;
    case ROT_RETRACT_258_RESTORE:
      if(OP_GOTOGRID==3)
      {
        servos_write(GRID[2-1].rot,-1,-1);
      }
      else if(OP_GOTOGRID==6)
      {
        servos_write(GRID[5-1].rot,-1,-1);
      }
      else if(OP_GOTOGRID==9)
      {
        servos_write(GRID[8-1].rot,-1,-1);
      }
      break;
    case PUT_258_RESTORE:
      if(OP_GOTOGRID==3)
      {
        servos_write(GRID[2-1].rot,GRID[2-1].dist,GRID[2-1].dist);
      }
      else if(OP_GOTOGRID==6)
      {
        servos_write(GRID[5-1].rot,GRID[5-1].dist,GRID[5-1].dist);
      }
      else if(OP_GOTOGRID==9)
      {
        servos_write(GRID[8-1].rot,GRID[8-1].dist,GRID[8-1].dist);
      }
      break;
    case OPEN_CLAW_258_RESTORE:
      PWM_Instance4->setPWM(Servo4_PIN, PWM_frequency, MAXSERVO4);
      break;
    case PUT_RETRACT_258_RESTORE:
      if(OP_GOTOGRID==3)
      {
        servos_write(GRID[2-1].rot,-1,-1);
      }
      else if(OP_GOTOGRID==6)
      {
        servos_write(GRID[5-1].rot,-1,-1);
      }
      else if(OP_GOTOGRID==9)
      {
        servos_write(GRID[8-1].rot,-1,-1);
      }
      break;
    default:
      break;
    }
    if(cs_main==CONFIG)
    {
      servos_write(config_rot,config_dist,config_dist);
      PWM_Instance4->setPWM(Servo4_PIN, PWM_frequency, MINSERVO4);
    }
    if(cs_robotic_arm==WAIT && cs_main==OPERATION)
    {
      servos_init_pos();
    }
    
  }
  
    
  /*
    SERVO_PRED_POS exemple1={0,0};
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

      if (Servo2_DC > MAXSERVO2) Servo2_DC = MAXSERVO2;
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
      
      /*show_lux = 1;
      uint16_t r, g, b, c, colorTemp, lux;
      getRawData_noDelay(&r, &g, &b, &c);
      colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
      if (show_lux) lux = tcs.calculateLux(r, g, b);

      if(now - last_cycle2 > 1000)
      {
        last_cycle2=now;
        
       /* Serial.print("Color : ");
        Serial.print(closestColor(r,g,b));
        Serial.print("\n");

        if (tof.readRangeAvailable()) {
        prev_distance = distance;
        distance = tof.readRangeMillimeters() * 1e-3;
        }
        tof.startReadRangeMillimeters(); 
        Serial.print("Dist: ");
        Serial.print(distance, 3);
        Serial.println();
        

        Serial.print("S1: ");
        Serial.print(Servo1_DC, 3);
        Serial.print(" S2: ");
        Serial.print(Servo2_DC, 3);
        Serial.print(" S3: ");
        Serial.print(Servo3_DC, 3);
        Serial.print(" S4: ");
        Serial.print(Servo4_DC, 3);
        Serial.print("\n");
      }

      PWM_Instance1->setPWM(Servo1_PIN, PWM_frequency, Servo1_DC);
      PWM_Instance2->setPWM(Servo2_PIN, PWM_frequency, Servo2_DC);
      PWM_Instance3->setPWM(Servo3_PIN, PWM_frequency, Servo3_DC);
      PWM_Instance4->setPWM(Servo4_PIN, PWM_frequency, Servo4_DC);
      
    }*/
}