/*
  Start code for haptic device testing
  Buliding the pwm motor liabry for inegrated control
*/

#include "pwmmotor.h"

// intitiate ports numbers
int motorSettings[5] = {7, 8, 9, 0, 1}; // Pins for motor {forward, backward, PWM, hall_1, hall_2}
int sensorSetting[2] = {6,7}; // Pins for sensor  {force, current}
int positionPID[3] = {1,0,0}; // Pid for position {Kp, Ki, Kd}
int forcePID[3] = {1,0,0}; // Pid for force {Kp, Ki, Kd} 


void setup() {
  // Setup of DC motor controller
  //pinMode(motorPinForward, OUTPUT);
  //pinMode(motorPinBackward, OUTPUT);

}

void loop() {
  //Motorcontrol_.goToPos(90);
}
