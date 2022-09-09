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

    float goImpedance(float massConstant, float damperConstant, float springConstant, float initialPosition = 125);
    float goAdmittance(float massConstant, float damperConstant, float springConstant, float initialPosition = -1 , float initialForce = 0);
  
    void studentProgram();

  private:
    // Needed objects are initialized
    HapticSensor ArmSensor_;
    pwmMotor MainMotor_;
    PID PositionPID_;
    PID MotorPID_;
    PID ForcePID_;

    // Calculation of moved length and angle and needed varriables for these two
    void movedLength();
    void movedAngle();
    float _lastPosition;
    float _lastMovedSpeed;
    float _lastAngle;
    float _lastMovedAngleSpeed;
    int _calibrationSpeed;
    float _armLength;
    unsigned long _mark_time;
    float _PosVelAks[3] = {0.0, 0.0, 0.0};
    float _AngVelAks[3] = {0.0, 0.0, 0.0};

    // Position values for imp/ad control
    float _currentAngAd;
    float _currentTorqueImp;

    // Switch and saturation values
    int _switchType; // NO = 1; NC = 0;
    int _saturationLimit[2] = {-255, 255};
    int _forward_index;

    // emergecy methods for stop of motor
    void emergencyCheck();
    void emergencyBreak();    
};

#endif 
