/*
  Start code for haptic device testing
  Buliding the pwm motor liabry for inegrated control
*/

#include "hapticarm.h"

// intitiate ports numbers
int motorSetting[5] = {7, 8, 9, 0, 1}; // Pins for motor {forward, backward, PWM, hall_1, hall_2}
int sensorSetting[4] = {10,6, 2, 3}; // Pins for sensor  {force, current, switch one, switch two}
float positionPID[3] = {2,0.001,1}; // Pid for position {Kp, Ki, Kd}
float forcePID[3] = {2,0,1}; // Pid for force {Kp, Ki, Kd} 

HapticArm HapArm_(motorSetting, sensorSetting, positionPID, forcePID);

void setup() {
  // Initislize importent comunication
  Serial.begin(9600);
  Wire.begin();
  HapArm_.calibrateArm();
  delay(500);
}

void loop() {
  HapArm_.goToPos(50);
  //HapArm_.resistForce(1.0);
  //HapArm_.goSpring(0,0,0,0);
  delay(2);
}
