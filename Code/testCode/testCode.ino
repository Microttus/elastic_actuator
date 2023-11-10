/*
* Author:
* Microttus
*
* Start code for haptic device testing
*  Buliding the pwm motor liabry for inegrated control
*
*/
#include "hapticarm.h"

#include "hapticsensor.h"

// Initiate ports numbers
int motorSetting[5] = {7, 8, 9, 2, 3};    // Pins for motor {forward, backward, PWM, hall_1, hall_2}
int sensorSetting[4] = {1, 0, 6, 10};      // Pins for sensor  {force, current, switch one, switch two}

// PID settings(Kp, Ki, Kd) {position, force, motor} 
float PIDset[3][3] = {{3,3,30},{3,1,0},{0,0,0}};

HapticArm HapArm_(motorSetting, sensorSetting, PIDset);

// Sinus wave
float A = -30;
float B = 0.1;
float dt = 0.0;
  
void setup() {
  // Initislize importent comunication
  Serial.begin(9600);
  Wire.begin();
  delay(2000);
  HapArm_.calibrateArm(); // If calibration is needed, else should be let out 
  delay(500);

  float curPos = 0;
  float startPos = 125;
  while (curPos != startPos && dt < 10){
    HapArm_.goToPos(startPos);
    dt = dt + 0.001;
  }
  dt = 0.0;
  
  //Admittance header
  //Serial.println("AcutalPos;SetPos;SpringAngle;Force;RequestedPos;");

  //Impedace header
  Serial.println("AcutalPos;SetPos;MotorReqSpeed;TorqueMes;TorqueCalc;");
  
}

void loop() {
  // sinewave for setopint reference
  float pos = A*sin(B*dt) + 125;

  // For control with Impedance uncommen this line
  //HapArm_.goImpedance(0, 0, 1);

  // For control with Adimttance uncomment this line
  //HapArm_.goAdmittance(0.1,2,0.02,pos);

  // For basic position control uncomment this line
  HapArm_.goToPos(125);

  // This function can be used if more sofiticated contorl functons are to be implimented
  //HapArm_.studentProgram();
}
