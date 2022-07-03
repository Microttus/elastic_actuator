/*
 * Author:
 * Microttus
 * 
 * 
 * This liabry requires: arduino
 */

#ifndef HapticArm_h
#define HapticArm_h

#include "Arduino.h"
#include "pid.h"
#include "hapticsensor.h"
#include "pwmmotor.h"

class HapticArm{
  public:
    HapticArm(int motorSettings[], int sensorSettings[], int PosSet[], int ForceSet[]);
    void goToPos(float requiredPos);
    void resistForce(float forceThreshold);
  
  private:
    HapticSensor ArmSensor_;
    pwmMotor MainMotor_;
    PID PositionPID_;
    PID ForcePID_;
    
};

#endif 
