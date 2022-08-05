/*
 * Main control libary with simple comands for controlling tha Haptic Arm
 *
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
    HapticArm(int motorSettings[], int sensorSettings[], float PIDset[][3]);

    void goToPos(float requiredPos);
    void calibrateArm();

    void goImpedance(float massConstant, float damperConstant, float springConstant);
    void goAdmittance(float massConstant, float damperConstant, float springConstant, float initialForce);
  
  private:
    // Needed objects are initialized
    HapticSensor ArmSensor_;
    pwmMotor MainMotor_;
    PID PositionPID_;
    PID MotorPID_;
    PID ForcePID_;

    // Calculation of moved length and angle and needed varriables for these two
    float* movedLength();
    float* movedAngle();
    float lastPosition;
    float lastMovedSpeed;
    float lastAngle;
    float lastMovedAngleSpeed;
    int calibrationSpeed;
    float armLength;
    unsigned long my_time;
    unsigned long dt;

    // Position values for imp/ad control
    float currentAngAd;
    float currentTorqueImp;

    // Switch and saturation values
    int raw_min;
    int raw_max;
    int switchType; // NO = 1; NC = 0;
    int saturationLimit[2] = {-255, 255};
    int forward_index;

    // emergecy methods for stop of motor
    void emergencyCheck();
    void emergencyBreak();    
};

#endif 
