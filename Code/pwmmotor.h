/*
 * pwmMotor
 * --------
 * for HapticSommerSchool
 * --------
 * Author: Microttus
 * 
 * 
 * A compeate class for control of a pwm motor with hall encoder
 * as an object, for easy operation
 * 
 * Required libaries: Arduino
 */

#ifndef pwmmotor_h
#define pwmmotor_h

#include "Arduino.h"

class pwmMotor{
  public:
    pwmMotor(int forwardPin_in, int backwardPin_in, int pwm_in, int hallPinOne_in, int hallPinTwo_in);
    void goToSpeed(int motorSpeed);           // Method primaly used for controlling the pwm motor
    void stop();                              // Total stop off the motor
    
    int check_rotation();                     // Method for returning the angle aproximated by the motor 
    static void reset_hall_val();             // Resett the hall counter for calibration
    int return_motor_dir();                   // Return the current motor dir

  private:
    static void increase_hall_val_one();      // Increase counter for hall one if triggered
    static void increase_hall_val_two();      // Increase counter for hall two if triggered

    int _forwardPin;
    int _backwardPin;
    int _PWMPin;
    int _hallOnePin;
    int _hallTwoPin;
    float _pass_by_rot;
    int _lastMotorSpeed;
    
    int _triggersPerRound;
    float _roundPerangle;
    int _saturationMax;
    int _saturationMin;
    
    bool _dirFlag;    
    float _newPos;
};

#endif
