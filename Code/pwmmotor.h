#ifndef pwmmotor_h
#define pwmmotor_h

#include "Arduino.h"
#include "pid.h"
#include "hapticsensor.h"

class pwmMotor{
  public:
    pwmMotor(int forwardPin_in, int backwardPin_in, int pwm_in, int hallPinOne_in, int hallPinTwo_in);
    void goToSpeed(int motorSpeed);
    int check_rotation();
    static void increase_hall_val_one();
    static void increase_hall_val_two();

  private:
    int forwardPin;
    int backwardPin;
    int PWMPin;
    int hallOnePin;
    int hallTwoPin;
    int pass_by_rot;
    int lastMotorSpeed;
    
    int triggersPerRound;
    float roundPerangle;
    int saturationMax;
    int saturationMin;
    
    bool dirFlag;
    
    float newPos;
    
    
    HapticSensor armSensor_(int a = 1, int b = 2);
};

#endif
