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
    HapticArm(int motorSettings[], int sensorSettings[], float PosSet[], float ForceSet[]);
    void goToPos(float requiredPos);
    void resistForce(float forceThreshold);
    void goSpring(float massConstant, float damperConstant, float springConstant, float initialPosition = 90.0);
    void calibrateArm();
  
  private:
    HapticSensor ArmSensor_;
    pwmMotor MainMotor_;
    PID PositionPID_;
    PID ForcePID_;

    float armLength;
    unsigned long my_time;
    unsigned long dt;

    float* calcMovement();
    float lastPosition;
    float lastMovedSpeed;
    int calibrationSpeed;
    int raw_min;
    int raw_max;
    int switchType; // NO = 1; NC = 0;
    
};

#endif 
