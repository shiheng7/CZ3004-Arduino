#include "PinChangeInt.h"
#include "SharpIR.h"
#include <DualVNH5019MotorShield.h>
#include <math.h>

DualVNH5019MotorShield md;

#define motor_encoder_R 11  // motor M1 // E2
#define motor_encoder_L 3   // motor M2 // E1

#define SRmodel 1080
#define LRmodel 20150
#define s1 A0 //front_left      //PS1
#define s2 A1 //front_middle    //PS2
#define s3 A2 //front_right     //PS3
#define s4 A3 //side_left_short //PS4
#define s5 A4 //side_left_long  //PS5
#define s6 A5 //side_right      //PS6

SharpIR sr1 =  SharpIR(s1, SRmodel);  //front_left      //PS1
SharpIR sr2 =  SharpIR(s2, SRmodel);  //front_middle    //PS2
SharpIR sr3 =  SharpIR(s3, SRmodel);  //front_right     //PS3
SharpIR sr4 =  SharpIR(s4, SRmodel);  //side_left_short //PS4
SharpIR lr5 =  SharpIR(s5, LRmodel);  //side_left_long  //PS5
SharpIR sr6 =  SharpIR(s6, SRmodel);  //side_right      //PS6

// ==============================   Global Variables  ==============================
int encoder_val_R = 0, encoder_val_L = 0;
int counter_MR = 0, counter_ML = 0;
int currentTime;
unsigned long startTime_MR, endTime_MR, duration_MR;
unsigned long startTime_ML, endTime_ML, duration_ML; 
String fastestString, calibrateString;
char header, func;

// ==============================   Working Variables   ==============================
double input_MR = 0, input_ML = 0;
double pidOutput_MR = 0, pidOutput_ML = 0;
double prev_pidOutput_MR = 0, prev_pidOutput_ML = 0;
double error_MR = 0, error_ML = 0;
double prev_error_MR = 0, prev_error_ML = 0;
double prev_prev_error_MR = 0, prev_prev_error_ML = 0;
double total_Dis = 0;

// ==============================   User Input Variables  ==============================
float kp_MR = 0.115;
float ki_MR = 0.420;
float kd_MR = 0.014;
float kp_ML = 0.081;
float ki_ML = 0.340;
float kd_ML = 0.014;
double setpoint = 80;
double setpoint_RT = 80;

float MIN_DISTANCE_CALIBRATE = 8.9;
float ANGLE_CALIBRATION_THRESHOLD = 0.05;
float LEFT_TO_WALL_DISTANCE_THRESHOLD[2] = {MIN_DISTANCE_CALIBRATE - 0.2, MIN_DISTANCE_CALIBRATE + 0.2};  // calDistance()
float RIGHT_TO_WALL_DISTANCE_THRESHOLD[2] = {MIN_DISTANCE_CALIBRATE - 0.2, MIN_DISTANCE_CALIBRATE + 0.2}; // calDistance()
float FRONT_TO_WALL_DISTANCE_THRESHOLD[2] = {MIN_DISTANCE_CALIBRATE - 0.2, MIN_DISTANCE_CALIBRATE + 0.2}; // calDistanceFront()
const int AVERAGE_FRONT = 0;
const int AVERAGE_READINGS = 1;
const int AVERAGE_DISCRETE = 2;
const int AVERAGE_FRONT_DISCRETE = 3;
int SENSOR_READING_TYPE = AVERAGE_FRONT_DISCRETE;
const int FRONT_LEFT_SENSOR = 1;
const int FRONT_MIDDLE_SENSOR = 2;
const int FRONT_RIGHT_SENSOR = 3;
const int LEFT_SHORT_SENSOR = 4;
const int LEFT_LONG_SENSOR = 5;
const int RIGHT_SHORT_SENSOR = 6;
const int NUM_SAMPLES = 10;

// ==============================   Setup   ==============================
void setup() {
  Serial.begin(115200);

  pinMode(motor_encoder_R, INPUT);
  pinMode(motor_encoder_L, INPUT);

  md.init();

  PCintPort::attachInterrupt(motor_encoder_R, countPulse_MR, RISING);
  PCintPort::attachInterrupt(motor_encoder_L, countPulse_ML, RISING);
  
  Serial.flush();
}

// ==============================   Loop  ==============================
void loop() {
  if(Serial.available() > 0) {
    header = char(Serial.read());
    if(header == '0') {
      while(Serial.available() <= 0){};
      func = char(Serial.read());
      readExploreCommands(func);
    }
    else if(header == '1') {
      fastestString = Serial.readStringUntil('\n');
      fastestString = checkFastestCommands(fastestString);
    }
    else if(header == '2') {
      readFastestCommands(fastestString);
    }
    else if(header == '6') {
      calibrateString = Serial.readStringUntil('\n');
      readCalibrateCommands(calibrateString);
    }
  } 
  restartPID();
}

// ==============================   Main Functions  ==============================
void readExploreCommands(char func) {
  //new shit
  restartPID();
  switch(func) {
    case 'L':
      rotateLeft();
//      if(getShortSensorString(aggregateDiscreteShortSensorGrids(1)) == "0")
//        calDistanceFront(MIN_DISTANCE_CALIBRATE, 1);
//      else if(getShortSensorString(aggregateDiscreteShortSensorGrids(2)) == "0")
//        calDistanceFront(MIN_DISTANCE_CALIBRATE, 2);
//      else if(getShortSensorString(aggregateDiscreteShortSensorGrids(3)) == "0")
//        calDistanceFront(MIN_DISTANCE_CALIBRATE, 3);
      Serial.print("5" + getSensorReadings() + "\n");
      delay(125);
      restartPID();  
      break;
    case 'R':
      rotateRight();
//      if(getShortSensorString(aggregateDiscreteShortSensorGrids(1)) == "0")
//        calDistanceFront(MIN_DISTANCE_CALIBRATE, 1);
//      else if(getShortSensorString(aggregateDiscreteShortSensorGrids(2)) == "0")
//        calDistanceFront(MIN_DISTANCE_CALIBRATE, 2);
//      else if(getShortSensorString(aggregateDiscreteShortSensorGrids(3)) == "0")
//        calDistanceFront(MIN_DISTANCE_CALIBRATE, 3);
      Serial.print("5" + getSensorReadings() + "\n");  
      delay(125);
      restartPID();
      break;
    case 'M':
      goStraightInGrids(1);
//      if(getShortSensorString(aggregateDiscreteShortSensorGrids(1)) == "0")
//        calDistanceFront(MIN_DISTANCE_CALIBRATE, 1);
//      else if(getShortSensorString(aggregateDiscreteShortSensorGrids(2)) == "0")
//        calDistanceFront(MIN_DISTANCE_CALIBRATE, 2);
//      else if(getShortSensorString(aggregateDiscreteShortSensorGrids(3)) == "0")
//        calDistanceFront(MIN_DISTANCE_CALIBRATE, 3);
      Serial.print("5" + getSensorReadings() + "\n");  
      restartPID();
      break;
    case 'B':
      goBackInGrids(1);
      Serial.print("5" + getSensorReadings() + "\n");  
      restartPID();
      break;
    case 'S':
      while(true) {
        Serial.print(getDistance(FRONT_LEFT_SENSOR)); Serial.print("<-- FL | ");
        Serial.print(getDistance(FRONT_MIDDLE_SENSOR)); Serial.print("<-- FC | ");
        Serial.print(getDistance(FRONT_RIGHT_SENSOR)); Serial.print("<-- FR | ");
        Serial.print(getDistance(LEFT_SHORT_SENSOR)); Serial.print("<-- LS | ");
        Serial.print(getDistance(LEFT_LONG_SENSOR)); Serial.print("<-- LL | ");
        Serial.print(getDistance(RIGHT_SHORT_SENSOR)); Serial.print("<-- RS\n");
        //delay(500);
      }
      break;
    case 'Q':
      while(true) {
        Serial.print(sr1.distance()); Serial.print("<-- FL | ");
        Serial.print(sr2.distance()); Serial.print("<-- FC | ");
        Serial.print(sr3.distance()); Serial.print("<-- FR | ");
        Serial.print(sr4.distance()); Serial.print("<-- LS | ");
        Serial.print(lr5.distance()); Serial.print("<-- LL | ");
        Serial.print(sr6.distance()); Serial.print("<-- RS\n");
        //delay(1000);
      }
    case 'G':
      //while(true) {
        Serial.print(getShortSensorString(aggregateContinuousShortSensorGrids(1))); Serial.print("<-- FL | ");
        Serial.print(getShortSensorString(aggregateContinuousShortSensorGrids(2))); Serial.print("<-- FC | ");
        Serial.print(getShortSensorString(aggregateContinuousShortSensorGrids(3))); Serial.print("<-- FR | ");
        Serial.print(getShortSensorString(aggregateContinuousShortSensorGrids(4))); Serial.print("<-- LS | ");
        Serial.print(getLongSensorString(aggregateContinuousLongSensorGrids(5))); Serial.print("<-- LL | ");
        Serial.print(getShortSensorString(aggregateContinuousShortSensorGrids(6))); Serial.print("<-- RS\n");
        //delay(1000);
      //}
      break;
    default:
      Serial.print("5" + getSensorReadings() + "\n");  
      break;
  }
}

void readCalibrateCommands(String calibrateString) {
  int stringLength = calibrateString.length();
  char func;

  for(int loop = 0; loop < stringLength; loop++) {
    func = calibrateString.charAt(loop);
    //new shit
    restartPID();
    switch(func) {
      case 'L':
        rotateLeft();
        delay(125);
        restartPID();
        break;
      case 'R':
        rotateRight();
        delay(125);
        restartPID();
        break;
      case 'C':
        calibrate(MIN_DISTANCE_CALIBRATE);
        delay(50);
        restartPID();
        break;
      default:
        break;
    }
  }
}

String checkFastestCommands(String fastestString) {
  int stringLength = fastestString.length();
  char func;
  
  for(int loop = 0; loop < stringLength; loop++) {
    func = fastestString.charAt(loop);
    //new shit
    restartPID();
    if(func == 'L' || func == 'R') {
      switch(func) {
      case 'L':
        rotateLeft();
        delay(125);
        restartPID();
        break;
      case 'R':
        rotateRight();
        delay(125);
        restartPID();
        break;
      default:
        break;
      }
    }
    else {
      fastestString.remove(0, loop);
      break;
    }
  }
  return fastestString;
}

void readFastestCommands(String fastestString) {
  int stringLength = fastestString.length();
  char func;

  for(int loop = 0; loop < stringLength; loop++) {
    func = fastestString.charAt(loop);
    //new shit
    restartPID();
    switch(func) {
      case 'L':
        rotateLeft();
        delay(125);
        restartPID();
        break;
      case 'R':
        rotateRight();
        delay(125);
        restartPID();
        break;
      default:
        int hexChar = int(func);
        if(hexChar >= 48 && hexChar <= 57){
          goStraightInGrids(hexChar - 48);
          delay(125);
          restartPID();
        }else if(hexChar >= 65 && hexChar <= 70){
          goStraightInGrids(hexChar - 55);
          delay(125);
          restartPID();
        }
        break;
    }
  }
}

void wait(unsigned long milliseconds) {
  currentTime = millis();
  while(millis() < currentTime + milliseconds) {}; 
}

// ==============================   Movement Functions  ==============================
void goStraightInGrids(long grids) {
  long distance = grids * 10600; 
  while(true) {
    if (total_Dis >= distance) {
      total_Dis = 0;
      md.setBrakes(400, 400);
      break;
    }
    else {
      moveForward();
      //Serial.print(input_MR); Serial.print(", R  ||  ");
      //Serial.print(input_ML); Serial.print(", L\n");
      total_Dis = total_Dis + input_ML + input_MR;
    }
  }
}

// new shit
void goBackInGrids(long grids) {
  long distance = grids * 10600; 
  while(true) {
    if (total_Dis >= distance) {
      total_Dis = 0;
      md.setBrakes(400, 400);
      break;
    }
    else {
      moveBackward();
      //Serial.print(input_MR); Serial.print(", R  ||  ");
      //Serial.print(input_ML); Serial.print(", L\n");
      total_Dis = total_Dis + input_ML + input_MR;
    }
  }
}

void rotateLeft() {
  long limit = 13850;
  while(true) {
    if (total_Dis >= limit) {
      total_Dis = 0;
      md.setBrakes(400, 400);
      break;
    }
    else {
      pidCalculation(kp_MR, ki_MR, kd_MR, kp_ML, ki_ML, kd_ML, setpoint_RT);
      md.setSpeeds(pidOutput_MR * 150, pidOutput_ML * 150);
      delayMicroseconds(5000);
      //Serial.print(input_MR); Serial.print(", R  ||  ");
      //Serial.print(input_ML); Serial.print(", L\n");
      total_Dis = total_Dis + input_ML + input_MR;
    }
  }
}

void rotateRight() {
  long limit = 13850;
  while(true) {
    if (total_Dis >= limit) {
      total_Dis = 0;
      md.setBrakes(400, 400);
      break;
    }
    else {
      pidCalculation(kp_MR, ki_MR, kd_MR, kp_ML, ki_ML, kd_ML, setpoint_RT);
      md.setSpeeds(-pidOutput_MR * 150, -pidOutput_ML * 150);
      delayMicroseconds(5000);
      //Serial.print(input_MR); Serial.print(", R  ||  ");
      //Serial.print(input_ML); Serial.print(", L\n");
      total_Dis = total_Dis + input_ML + input_MR;
    }
  }
}

// ==============================   Motor Functions   ==============================
void countPulse_MR() {
  counter_MR++;
  if(counter_MR == 1) startTime_MR = micros();
  else if(counter_MR == 11) {
    endTime_MR = micros();
    duration_MR = (endTime_MR - startTime_MR) / 10.0;
    input_MR = calculateRPM(duration_MR);
    counter_MR = 0;
  }
}

void countPulse_ML() {
  counter_ML++;
  if(counter_ML == 1) startTime_ML = micros();
  else if(counter_ML == 11) {
    endTime_ML = micros();
    duration_ML = (endTime_ML - startTime_ML) / 10.0;
    input_ML = calculateRPM(duration_ML);
    counter_ML = 0;
  }
}

double calculateRPM(double pulse) {
  if (pulse == 0) return 0;
  else return 60.00 / (pulse *  562.25 / 1000000.00);
}

void pidCalculation(float kp_MR, float ki_MR, float kd_MR, float kp_ML, float ki_ML, float kd_ML, double setpoint) {

  // calculate k1, k2, k3 for both motors
  double k1_MR = kp_MR + ki_MR + kd_MR;
  double k2_MR = -kp_MR - 2 * kd_MR;
  double k3_MR = kd_MR;

  double k1_ML = kp_ML + ki_ML + kd_ML;
  double k2_ML = -kp_ML - 2 * kd_ML;
  double k3_ML = kd_ML;

  // calculate error values
  error_MR = (setpoint - input_MR) / 130.0;
  error_ML = (setpoint - input_ML) / 130.0;

  // compute PID controller output
  pidOutput_MR = prev_pidOutput_MR + k1_MR * error_MR + k2_MR * prev_error_MR + k3_MR * prev_prev_error_MR;
  pidOutput_ML = prev_pidOutput_ML + k1_ML * error_ML + k2_ML * prev_error_ML + k3_ML * prev_prev_error_ML;

  // save current data into memory
  prev_prev_error_MR = prev_error_MR;
  prev_prev_error_ML = prev_error_ML;
  prev_error_MR = error_MR;
  prev_error_ML = error_ML;
  prev_pidOutput_MR = pidOutput_MR;
  prev_pidOutput_ML = pidOutput_ML;
}

void restartPID() {
  prev_prev_error_MR = prev_prev_error_ML = 0;
  prev_error_MR = prev_error_ML = 0;
  error_MR = error_ML = 0;
  prev_pidOutput_MR = prev_pidOutput_ML = 0;
  pidOutput_MR = pidOutput_ML = 0;
  input_MR = input_ML = 0;
}

void moveForward() {
  pidCalculation(kp_MR, ki_MR, kd_MR, kp_ML, ki_ML, kd_ML, setpoint);
  md.setSpeeds(pidOutput_MR * 150, -pidOutput_ML * 150);
  delayMicroseconds(5000);
}

// new shit
void moveBackward() {
  pidCalculation(kp_MR, ki_MR, kd_MR, kp_ML, ki_ML, kd_ML, setpoint);
  md.setSpeeds(-pidOutput_MR * 150, pidOutput_ML * 150);
  delayMicroseconds(5000);
}

// ==============================   Sensor Functions  ==============================
double getDistance(int sensor) {
  double sval;
  switch(sensor){
    case 1:
      sval = sr1.distance();
      if(sval >= 0 && sval < 10)  return sval;
      if(sval >= 10 && sval < 12)  return sval + 0.20;
      if(sval >= 12 && sval < 14)  return sval - 0.10;
      if(sval >= 14 && sval < 16)  return sval - 0.10;
      if(sval >= 16 && sval < 18)  return sval - 0.60;
      if(sval >= 18 && sval < 20)  return sval - 0.80;
      if(sval >= 20 && sval < 22.5)  return sval - 1.20;
      if(sval >= 22.5 && sval < 25)  return sval - 1.40;
      if(sval >= 25 && sval < 30)  return sval - 2.20;
      if(sval >= 30 && sval < 35)  return sval - 3.50;
      if(sval >= 35 && sval < 37)  return sval - 4.00; //accurate up to 30cm
      if(sval >= 37) return 99.99;
      break;
    case 2:
      sval = sr2.distance();
      if(sval >= 0 && sval < 10)  return sval;
      if(sval >= 10 && sval < 12)  return sval + 1.00;
      if(sval >= 12 && sval < 14)  return sval + 1.60;
      if(sval >= 14 && sval < 16)  return sval + 1.70;
      if(sval >= 16 && sval < 18)  return sval + 2.30;
      if(sval >= 18 && sval < 20)  return sval + 2.80;
      if(sval >= 20 && sval < 22.5)  return sval + 4.90;
      if(sval >= 22.5 && sval < 24)  return sval + 5.50;
      if(sval >= 24)return 99.99;
      break;
    case 3:
      sval = sr3.distance();
      if(sval >= 0 && sval < 10)  return sval - 0.20;
      if(sval >= 10 && sval < 12)  return sval - 0.40;
      if(sval >= 12 && sval < 14)  return sval - 0.90;
      if(sval >= 14 && sval < 16)  return sval - 0.90;
      if(sval >= 16 && sval < 18)  return sval - 1.10;
      if(sval >= 18 && sval < 20)  return sval - 1.80;
      if(sval >= 20 && sval < 22.5)  return sval - 1.80;
      if(sval >= 22.5 && sval < 25)  return sval - 1.80;
      if(sval >= 25 && sval < 30)  return sval - 2.00;
      if(sval >= 30 && sval < 35)  return sval - 2.30;
      if(sval >= 35)  return 99.99; //accurate up to 30cm
      break;
    case 4:
      sval = sr4.distance();
      if(sval >= 0 && sval < 10) return sval + 0.50;
      if(sval >= 10 && sval < 12)  return sval + 0.40;
      if(sval >= 12 && sval < 14)  return sval + 0.40;
      if(sval >= 14 && sval < 16)  return sval + 0.30;
      if(sval >= 16 && sval < 18)  return sval + 0.10;
      if(sval >= 18 && sval < 20)  return sval + 0.05; 
      if(sval >= 20 && sval < 25)  return sval + 0.05;
      if(sval >= 25 && sval < 30)  return sval + 0.05;
      if(sval >= 30) return 99.99;
      break;
    case 5:
      sval = lr5.distance();
      if(sval >= 0 && sval < 19) return 0;
      if(sval >= 19 && sval < 22)  return sval + 1.00;
      if(sval >= 22 && sval < 24)  return sval + 2.43;
      if(sval >= 24 && sval < 26)  return sval + 3.03;
      if(sval >= 26 && sval < 28)  return sval + 3.54;
      if(sval >= 28 && sval < 30)  return sval + 4.00;
      if(sval >= 30 && sval < 35)  return sval + 4.50;
      if(sval >= 35 && sval < 40)  return sval + 4.00;
      if(sval >= 40 && sval < 45)  return sval + 5.00;
      if(sval >= 45 && sval < 50)  return sval + 3.30; 
      if(sval >= 50 && sval < 55)  return sval + 4.30; 
      if(sval >= 55 && sval < 60)  return sval + 4.30; //accurate up to 60cm
      if(sval >= 60)  return 99.99;
      break;
    case 6:
      sval = sr6.distance();
      if(sval >= 0 && sval < 10) return sval;
      if(sval >= 10 && sval < 12)  return sval + 0.00;
      if(sval >= 12 && sval < 14)  return sval + 0.00;
      if(sval >= 14 && sval < 16)  return sval - 0.30;
      if(sval >= 16 && sval < 18)  return sval - 0.60;
      if(sval >= 18 && sval < 20)  return sval - 1.00; 
      if(sval >= 20 && sval < 25)  return sval - 1.20; 
      if(sval >= 25 && sval < 30)  return sval - 1.80; 
      if(sval >= 30) return 99.99;
      break;
    default:
      return 99.99;
  }
  return 99.99;
}

// get distance in grids for single reading
int getGridsFromDistanceShort(double distance) {  
    if(distance >= 4 && distance <= 14) return 1; // Actually 0
    else if(distance > 14 && distance <= 24) return 2; // Actually 1
    else if(distance > 24 && distance <= 34) return 3; // Actually 2
    else return 0; // Actually x
}

// Aggregate continuous sensor readings and get grids once
int aggregateContinuousShortSensorGrids(int sensor){
  int sum = 0;
  for(int i = 0; i < NUM_SAMPLES; i++){
    sum += getDistance(sensor);
  }
  return getGridsFromDistanceShort(sum/NUM_SAMPLES);
}

// Aggregate get grids to find majority case
int aggregateDiscreteShortSensorGrids(int sensor){
  int readings[] = {0,0,0,0};
  for(int i = 0; i < NUM_SAMPLES; i++){
    readings[getGridsFromDistanceShort(getDistance(sensor))] += 1;
  }
  int majority = 0;
  for(int i = 1; i < 4; i++){
    if(readings[i] > readings[majority]) majority = i;
  }
  return majority;
}

// Convert integer to equivalent string
String getShortSensorString(int reading){
  switch(reading){
    case 0:
      return "x";
    case 1:
      return "0";
    case 2:
      return "1";
    case 3:
      return "2";
  }
  return "x";
}

// get distance in grids for single reading
int getGridsFromDistanceLong(double distance) {  
  if(distance >= 0 && distance <= 24) return 1; // Actually x
    else if(distance > 24 && distance <= 34) return 2; // Actually 2
    else if(distance > 34 && distance <= 44) return 3; // Actually 3
    else if(distance > 44 && distance <= 54) return 4; // Actually 4
    else if(distance > 54 && distance <= 64) return 5; // Actually 5
    else return 0; // Actually x
}

// Aggregate continuous sensor readings and get grids once
int aggregateContinuousLongSensorGrids(int sensor){
    int sum = 0;
  for(int i = 0; i < NUM_SAMPLES; i++){
    sum += getDistance(sensor);
  }
  return getGridsFromDistanceLong(sum/NUM_SAMPLES);
}

// Aggregate get grids to find majority case
int aggregateDiscreteLongSensorGrids(int sensor){
  int readings[] = {0,0,0,0,0,0};
  for(int i = 0; i < NUM_SAMPLES; i++){
    readings[getGridsFromDistanceLong(getDistance(sensor))] += 1;
  }
  int majority = 0;
  for(int i = 1; i < 6; i++){
    if(readings[i] > readings[majority]) majority = i;
  }
  return majority;
}

// Convert integer to equivalent string
String getLongSensorString(int reading){
  switch(reading){
    case 0:
      return "x";
    case 1:
      return "x";
    case 2:
      return "2";
    case 3:
      return "3";
    case 4:
      return "4";
    case 5:
      return "5";
  }
  return "x";
}

String getSensorReadings(){
  switch(SENSOR_READING_TYPE){
    case AVERAGE_FRONT:
      return averageFrontErrorReadings();
    case AVERAGE_DISCRETE:
      return averageDiscreteReadings();
    case AVERAGE_FRONT_DISCRETE:
      return averageFrontErrorDiscreteReadings();
    case AVERAGE_READINGS:
      return averageContinuousReadings();
  }
}

// Takes front error and averages it before taking reading. Takes mode of 10 readings for all sensors.
String averageFrontErrorDiscreteReadings(){
  int FLarr[] = {0,0,0,0};
  int FCarr[] = {0,0,0,0};
  int FRarr[] = {0,0,0,0};
  int FLread[NUM_SAMPLES] = {};
  int FCread[NUM_SAMPLES] = {};
  int FRread[NUM_SAMPLES] = {};
  int frontAverageError = 0;
  double frontLeft, frontCenter, frontRight = 0;
  int count = 0;
  double measurement = 0;
  double modmeasurement = 0;
  for(int i = 0; i < NUM_SAMPLES; i++){
    measurement = getDistance(FRONT_LEFT_SENSOR);
    if(measurement < 90){
      modmeasurement = fmod(measurement, 10);
      if(modmeasurement < 5){
        frontAverageError += modmeasurement;
        FLread[i] = (int)(measurement-modmeasurement);
      }else{
        frontAverageError += modmeasurement - 10;
        FLread[i] = (int)(measurement-modmeasurement) + 10;
      }
      count += 1;
    }
    measurement = getDistance(FRONT_MIDDLE_SENSOR);
    if(measurement < 90){
      modmeasurement = fmod(measurement, 10);
      if(modmeasurement < 5){
        frontAverageError += modmeasurement;
        FCread[i] = (int)(measurement-modmeasurement);
      }else{
        frontAverageError += modmeasurement - 10;
        FCread[i] = (int)(measurement-modmeasurement) + 10;
      }
      count += 1;
    }
    measurement = getDistance(FRONT_RIGHT_SENSOR);
    if(measurement < 90){
      modmeasurement = fmod(measurement, 10);
      if(modmeasurement < 5){
        frontAverageError += modmeasurement;
        FRread[i] = (int)(measurement-modmeasurement);
      }else{
        frontAverageError += modmeasurement - 10;
        FRread[i] = (int)(measurement-modmeasurement) + 10;
      }
      count += 1;
    }
  }
  frontAverageError /= count;
  for(int i = 0; i < NUM_SAMPLES; i++){
    FLarr[getGridsFromDistanceShort(FLread[i]+frontAverageError)] += 1;
    FCarr[getGridsFromDistanceShort(FCread[i]+frontAverageError)] += 1;
    FRarr[getGridsFromDistanceShort(FRread[i]+frontAverageError)] += 1;
  }
  int maxFL = 0;
  int maxFC = 0;
  int maxFR = 0;
  for(int i = 1; i < 4; i++){
    if(FLarr[i] > FLarr[maxFL]){
      maxFL = i;
    }
    if(FCarr[i] > FCarr[maxFC]){
      maxFC = i;
    }
    if(FRarr[i] > FRarr[maxFR]){
      maxFR = i;
    }
  }
  String front_left = getShortSensorString(maxFL);
  String front_center = getShortSensorString(maxFC);
  String front_right = getShortSensorString(maxFR);
  String left_short = getShortSensorString(aggregateDiscreteShortSensorGrids(LEFT_SHORT_SENSOR));
  String left_long = getLongSensorString(aggregateDiscreteLongSensorGrids(LEFT_LONG_SENSOR));
  String right_short = getShortSensorString(aggregateDiscreteShortSensorGrids(RIGHT_SHORT_SENSOR));
  return left_short + left_long + front_left + front_center + front_right + right_short;
}

// Takes front error and averages it before taking single reading. Left and right sensors take mode of 10 discrete readings
String averageFrontErrorReadings(){
  double frontAverageError = 0;
  int count = 0;
  double measurement = 0;
  for(int i = 0; i < NUM_SAMPLES; i++){
    measurement = getDistance(FRONT_LEFT_SENSOR);
    if(measurement < 90){
      frontAverageError += fmod(measurement, 10) < 5? fmod(measurement, 10):fmod(measurement, 10)-10;
      count += 1;
    }
    measurement = getDistance(FRONT_MIDDLE_SENSOR);
    if(measurement < 90){
      frontAverageError += fmod(measurement, 10) < 5? fmod(measurement, 10):fmod(measurement, 10)-10;
      count += 1;
    }
    measurement = getDistance(FRONT_RIGHT_SENSOR);
    if(measurement < 90){
      frontAverageError += fmod(measurement, 10) < 5? fmod(measurement, 10):fmod(measurement, 10)-10;
      count += 1;
    }
  }
  frontAverageError /= count;
  double frontLeft = fmod(getDistance(FRONT_LEFT_SENSOR),10) < 5?
    (((int)getDistance(FRONT_LEFT_SENSOR)) / 10 * 10) + frontAverageError:
    (((int)getDistance(FRONT_LEFT_SENSOR)) / 10 * 10) + 10 + frontAverageError;
  String front_left = getShortSensorString(getGridsFromDistanceShort(frontLeft));
  double  frontCenter = fmod(getDistance(FRONT_MIDDLE_SENSOR),10) < 5?
    (((int)getDistance(FRONT_MIDDLE_SENSOR)) / 10 * 10) + frontAverageError:
    (((int)getDistance(FRONT_MIDDLE_SENSOR)) / 10 * 10) + 10 + frontAverageError;
  String front_center = getShortSensorString(getGridsFromDistanceShort(frontCenter));
  double frontRight = fmod(getDistance(FRONT_RIGHT_SENSOR),10) < 5?
    (((int)getDistance(FRONT_RIGHT_SENSOR)) / 10 * 10) + frontAverageError:
    (((int)getDistance(FRONT_RIGHT_SENSOR)) / 10 * 10) + 10 + frontAverageError;
  String front_right = getShortSensorString(getGridsFromDistanceShort(frontRight));
  String left_short = getShortSensorString(aggregateDiscreteShortSensorGrids(LEFT_SHORT_SENSOR));
  String left_long = getLongSensorString(aggregateDiscreteLongSensorGrids(LEFT_LONG_SENSOR));
  String right_short = getShortSensorString(aggregateDiscreteShortSensorGrids(RIGHT_SHORT_SENSOR));
  return left_short + left_long + front_left + front_center + front_right + right_short;
}

// Takes mode of 10 discrete readings for all sensors
String averageDiscreteReadings(){
  String front_left = getShortSensorString(aggregateDiscreteShortSensorGrids(FRONT_LEFT_SENSOR));
  String front_center = getLongSensorString(aggregateDiscreteLongSensorGrids(FRONT_MIDDLE_SENSOR));
  String front_right = getShortSensorString(aggregateDiscreteShortSensorGrids(FRONT_RIGHT_SENSOR));
  String left_short = getShortSensorString(aggregateDiscreteShortSensorGrids(LEFT_SHORT_SENSOR));
  String left_long = getLongSensorString(aggregateDiscreteLongSensorGrids(LEFT_LONG_SENSOR));
  String right_short = getShortSensorString(aggregateDiscreteShortSensorGrids(RIGHT_SHORT_SENSOR));
  return left_short + left_long + front_left + front_center + front_right + right_short;
}

// Takes average of 10 continuous readings for all sensors (original)
String averageContinuousReadings(){
  String front_left = getShortSensorString(aggregateContinuousShortSensorGrids(FRONT_LEFT_SENSOR));
  String front_center = getLongSensorString(aggregateContinuousLongSensorGrids(FRONT_MIDDLE_SENSOR));
  String front_right = getShortSensorString(aggregateContinuousShortSensorGrids(FRONT_RIGHT_SENSOR));
  String left_short = getShortSensorString(aggregateContinuousShortSensorGrids(LEFT_SHORT_SENSOR));
  String left_long = getLongSensorString(aggregateContinuousLongSensorGrids(LEFT_LONG_SENSOR));
  String right_short = getShortSensorString(aggregateContinuousShortSensorGrids(RIGHT_SHORT_SENSOR));
  return left_short + left_long + front_left + front_center + front_right + right_short;
}

// ==============================   Calibration Functions   ==============================
void calSensors(float error) {
  delay(1);
  //if the robot is tilted to the left -> need to turn right
  if(error > ANGLE_CALIBRATION_THRESHOLD) {
    md.setSpeeds(-100, -100);
    delay(abs(error * 50));
    md.setBrakes(400,400);
  }
  //if the robot is tilted to the right -> need to turn left 
  else if(error < -ANGLE_CALIBRATION_THRESHOLD) {
    md.setSpeeds(100, 100);
    delay(abs(error * 50));
    md.setBrakes(400, 400);    
  }
}

void calAngle(float MIN_DISTANCE_CALIBRATE) {
  float leftToWallDistance, rightToWallDistance = 0;
  int count = 0;
  float error;
  float forward = 0;
  while(true) {
    leftToWallDistance = getDistance(1);
    rightToWallDistance = getDistance(3);
    error = leftToWallDistance - rightToWallDistance;

    if(abs(error) <= ANGLE_CALIBRATION_THRESHOLD) break;

    //cannot calibrate because the robot is too far away
//    if(leftToWallDistance > (MIN_DISTANCE_CALIBRATE) && 
//       rightToWallDistance > (MIN_DISTANCE_CALIBRATE)) break;
    while(leftToWallDistance > MIN_DISTANCE_CALIBRATE && rightToWallDistance > MIN_DISTANCE_CALIBRATE){
      forward = leftToWallDistance < rightToWallDistance? (leftToWallDistance - MIN_DISTANCE_CALIBRATE) : (rightToWallDistance - MIN_DISTANCE_CALIBRATE);
      md.setSpeeds(75, -75);
      delay(abs(forward) * 250);
      md.setBrakes(400, 400);
      leftToWallDistance = getDistance(1);
      rightToWallDistance = getDistance(3);
    }

    //force a reverse because the robot is too near
    if(leftToWallDistance < MIN_DISTANCE_CALIBRATE && rightToWallDistance < MIN_DISTANCE_CALIBRATE) {
      md.setSpeeds(-75, 75);
      delay(abs(error) * 250);
      md.setBrakes(400, 400);
    }

    calSensors(error);
  }
  md.setBrakes(400, 400);
  delay(1);
}

void calDistance(float MIN_DISTANCE_CALIBRATE) {
  float leftToWallDistance, rightToWallDistance;
  float error;

  while(true) {
    leftToWallDistance = getDistance(1);
    rightToWallDistance = getDistance(3);
    error = MIN_DISTANCE_CALIBRATE - ((leftToWallDistance + rightToWallDistance) / 2);

    if(leftToWallDistance > MIN_DISTANCE_CALIBRATE || 
       rightToWallDistance > MIN_DISTANCE_CALIBRATE) break;

    //abort if within margin of error
    if ((leftToWallDistance >= LEFT_TO_WALL_DISTANCE_THRESHOLD[0] &&
         leftToWallDistance < LEFT_TO_WALL_DISTANCE_THRESHOLD[1]) || 
        (rightToWallDistance >= RIGHT_TO_WALL_DISTANCE_THRESHOLD[0] &&
         rightToWallDistance < RIGHT_TO_WALL_DISTANCE_THRESHOLD[1])) break;

    if (rightToWallDistance < RIGHT_TO_WALL_DISTANCE_THRESHOLD[0] || 
        leftToWallDistance < LEFT_TO_WALL_DISTANCE_THRESHOLD[0]) {
      //reverse normally
      md.setSpeeds(-200, 200);
      delay(abs(error) * 40);  //100
      md.setBrakes(400, 400);
    }
    else {  
      //move forward
      //moveForward(100,0.9);
      // Serial.println("Moving forwards");
      md.setSpeeds(200, -200);
      delay(abs(error) * 40);    //100
      md.setBrakes(400, 400);
    }
  }
}

void calDistanceFront(float MIN_DISTANCE_CALIBRATE, int sensor) {
  float sensorToWallDistance;
  float error;
     
  while(true) {
    sensorToWallDistance = getDistance(sensor);
    error = MIN_DISTANCE_CALIBRATE - sensorToWallDistance;

    //if(sensorToWallDistance > MIN_DISTANCE_CALIBRATE) break;

    //abort if within margin of error
    if ((sensorToWallDistance >= FRONT_TO_WALL_DISTANCE_THRESHOLD[0] &&
         sensorToWallDistance < FRONT_TO_WALL_DISTANCE_THRESHOLD[1])) break;

    if (sensorToWallDistance < FRONT_TO_WALL_DISTANCE_THRESHOLD[0]) {
      //reverse normally
      md.setSpeeds(-200, 200);
      delay(abs(error) * 40);  //100
      md.setBrakes(400, 400);
    }
    else {  
      //move forward
      //moveForward(100,0.9);
      // Serial.println("Moving forwards");
      md.setSpeeds(200, -200);
      delay(abs(error) * 40);    //100
      md.setBrakes(400, 400);
    }
  }
}

void calibrate(float MIN_DISTANCE_CALIBRATE) {
  float leftDist = getDistance(1);
  float rightDist = getDistance(3);
  if(leftDist > MIN_DISTANCE_CALIBRATE + 10 || rightDist > MIN_DISTANCE_CALIBRATE + 10) return;
  delay(1);
  calAngle(MIN_DISTANCE_CALIBRATE);
  delay(10);
  calDistance(MIN_DISTANCE_CALIBRATE);
  delay(10);
  calAngle(MIN_DISTANCE_CALIBRATE);
}

// ==============================   Miscellaneous/Obsolete/Testing Functions  ==============================
  /*
  switch(packet) {
    case '0':
      restartPID();
      while(true) {
        moveForward();
        Serial.print(input_MR); Serial.print(", R  ||  ");
        Serial.print(input_ML); Serial.print(", L\n");
      }
      break; 
    case '1':
      goStraightInGrids(1);
      restartPID();
      break;
    case '2':
      goStraightInGrids(2);
      restartPID();
      break;
    case 's':
      while(true) {
        Serial.print(getDistance(1)); Serial.print("<-- FL | ");
        Serial.print(getDistance(2)); Serial.print("<-- FC | ");
        Serial.print(getDistance(3)); Serial.print("<-- FR | ");
        Serial.print(getDistance(4)); Serial.print("<-- LS | ");
        Serial.print(getDistance(5)); Serial.print("<-- LL | ");
        Serial.print(getDistance(6)); Serial.print("<-- RS\n");
        //delay(500);
      }
      break;
    case 'g':
      while(true) {
        Serial.print(getDistanceInGrids(1)); Serial.print("<-- FL | ");
        Serial.print(getDistanceInGrids(2)); Serial.print("<-- FC | ");
        Serial.print(getDistanceInGrids(3)); Serial.print("<-- FR | ");
        Serial.print(getDistanceInGrids(4)); Serial.print("<-- LS | ");
        Serial.print(getDistanceInGrids(5)); Serial.print("<-- LL | ");
        Serial.print(getDistanceInGrids(6)); Serial.print("<-- RS\n");
        //delay(1000);
      }
      break;
    case 'l':
      rotateLeft(1);
      restartPID();
      break;
    case 'r':
      rotateRight(1);
      restartPID();
      break;
    case 'q':
      while(true) {
        Serial.print(sr1.distance()); Serial.print("<-- FL | ");
        Serial.print(sr2.distance()); Serial.print("<-- FC | ");
        Serial.print(sr3.distance()); Serial.print("<-- FR | ");
        Serial.print(sr4.distance()); Serial.print("<-- LS | ");
        Serial.print(lr5.distance()); Serial.print("<-- LL | ");
        Serial.print(sr6.distance()); Serial.print("<-- RS\n");
        //delay(1000);
      }
    case 'e':
      restartPID();
      explore();
      break;
    case 'f':
      restartPID();
      explore2();
      break;
    case 'c':
      calibrate(MIN_DISTANCE_CALIBRATE);
      delay(150);
      break;
    default:
      break;  
  }*/

/*
void explore() {
  for(int i = 0; i < 500; i++) {
    moveForward();
    Serial.print(i); Serial.print("\n");
    //Serial.print(input_MR); Serial.print(", R  ||  ");
    //Serial.print(input_ML); Serial.print(", L  ||  ");
    //Serial.print(getDistanceInGrids(1)); Serial.print("<-- FL | ");
    //Serial.print(getDistanceInGrids(2)); Serial.print("<-- FC | ");
    //Serial.print(getDistanceInGrids(3)); Serial.print("<-- FR\n");
  }

  while(true) {
    moveForward();
    if((getDistanceInGrids(1) == "1") || (getDistanceInGrids(2) == "1") || (getDistanceInGrids(3) == "1")) {
      md.setBrakes(400, 400);
      delay(500);
      rotateLeft(1);        
      delay(500);
      restartPID();
      delay(500);
      goStraightInGrids(3); 
      delay(500);
      rotateRight(1);       
      delay(500);
      restartPID();
      delay(500);
      goStraightInGrids(4); 
      delay(500);
      rotateRight(1);
      delay(500);
      restartPID();
      delay(500);
      goStraightInGrids(3); 
      delay(500);
      rotateLeft(1);        restartPID();
      delay(1000);
      break;     
    }
  }

  goStraightInGrids(2);
  restartPID();
}

void explore2() {
  for(int i = 0; i < 500; i++) {
    moveForward();
    Serial.print(i); Serial.print("\n");
    //Serial.print(input_MR); Serial.print(", R  ||  ");
    //Serial.print(input_ML); Serial.print(", L  ||  ");
    //Serial.print(getDistanceInGrids(1)); Serial.print("<-- FL | ");
    //Serial.print(getDistanceInGrids(2)); Serial.print("<-- FC | ");
    //Serial.print(getDistanceInGrids(3)); Serial.print("<-- FR\n");
  }

  while(true) {
    moveForward();
    if((getDistanceInGrids(1) == "2") || (getDistanceInGrids(2) == "2") || (getDistanceInGrids(3) == "2")) {
      md.setBrakes(400, 400);
      delay(500);
      rotateLeft(0.5);        
      delay(500);
      restartPID();
      delay(500);
      goStraightInGrids(3); 
      delay(500);
      rotateRight(0.5);       
      delay(500);
      restartPID();
      delay(500);
      goStraightInGrids(4); 
      delay(500);
      rotateRight(0.5);
      delay(500);
      restartPID();
      delay(500);
      goStraightInGrids(3); 
      delay(500);
      rotateLeft(0.5);        restartPID();
      delay(1000);
      break;     
    }
  }

  goStraightInGrids(2);
  restartPID();
}
*/
