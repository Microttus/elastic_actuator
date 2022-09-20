/*
 * Author:
 * Microttus
 *
 * An example of a simulated situation where the arm hit a ball at 150 deg and higher
 *
 * Include:
 * - HapticArm.h
 * 
 */

#include "hapticarm.h"

// INitiate global values
float currentAngle = 120;
float ballAngle = 150;

// Intitiate ports numbers
int motorSetting[5] = {7, 8, 9, 2, 3};    // Pins for motor {forward, backward, PWM, hall_1, hall_2}
int sensorSetting[4] = {1, 0, 6, 10};      // Pins for sensor  {force, current, switch one, switch two}

// Pid values as list
// PID for position {Kp, Ki, Kd}; PID for outer cascade motor {Kp, Ki, Kd}; PID for force {Kp, Ki, Kd} 
float PIDset[3][3] = {{1.4,0.001,1000},{0,0,0},{1,0,0.54}};

HapticArm HapArm_(motorSetting, sensorSetting, PIDset);

void setup() {
  // Initislize importent comunication
  Serial.begin(9600);
  Wire.begin();
  delay(2000);
  currentAngle = HapArm_.calibrateArm(); // If calibration is needed, else should be let out 
  delay(500);

}

void loop() {
  if (currentAngle < ballAngle) {
    currentAngle = goAdmittance(0.1,0,0);
  } else if (currentAngle >= ballAngle) {
    currentAngle = goAdmittance(0.1,2,10);
  }
  

}
