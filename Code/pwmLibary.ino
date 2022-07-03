/*
  Start code for haptic device testing
  Buliding the pwm motor liabry for inegrated control
*/

#include "hapticarm.h"

// intitiate ports numbers
int motorSetting[5] = {7, 8, 9, 0, 1}; // Pins for motor {forward, backward, PWM, hall_1, hall_2}
int sensorSetting[2] = {6,7}; // Pins for sensor  {force, current}
int positionPID[3] = {2,0,0}; // Pid for position {Kp, Ki, Kd}
int forcePID[3] = {1,0,0}; // Pid for force {Kp, Ki, Kd} 

HapticArm HapArm_(motorSetting, sensorSetting, positionPID, forcePID);

void setup() {
  // Initislize importent comunication
  Serial.begin(9600);
  Wire.begin();

}

void loop() {
  HapArm_.goToPos(90);
  delay(2);
}
