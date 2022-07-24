/*
* Author:
* Microttus
*
* Start code for haptic device testing
*  Buliding the pwm motor liabry for inegrated control
*
*/

#include "hapticarm.h"

// intitiate ports numbers
int motorSetting[5] = {7, 8, 9, 0, 1};    // Pins for motor {forward, backward, PWM, hall_1, hall_2}
int sensorSetting[4] = {10,6, 2, 3};      // Pins for sensor  {force, current, switch one, switch two}

float positionPID[3] = {2,0.001,1};       // PID for position {Kp, Ki, Kd}
float motorPID[3] = {1,0,0};              // PID for outer cascade motor {Kp, Ki, Kd}
float forcePID[3] = {2,0,1};              // PID for force {Kp, Ki, Kd} 

float PIDset[3][3] = {{0.8,0.001,2},{2,0,1},{1,0,0.54}};

HapticArm HapArm_(motorSetting, sensorSetting, PIDset);

void setup() {
  // Initislize importent comunication
  Serial.begin(9600);
  Wire.begin();
  delay(2000);
  HapArm_.calibrateArm(); // If calibration is needed, els should be let out 
  delay(500);
}

void loop() {
  //HapArm_.goToImpedance(120,0.01);
  HapArm_.goToPos(100);
  delay(2);
}
