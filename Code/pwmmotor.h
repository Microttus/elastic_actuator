#ifndef pwmmotor_h
#define pwmmotor_h

#include "Arduino.h"

class pwmMotor{
  public:
    pwmMotor(int forwardPin_in, int backwardPin_in, int pwm_in, int hallPinOne_in, int hallPinTwo_in);
    void goToPos(float requiredPos);
    static void increase_hall_vall_one();

  private:
    int forwardPin;
    int backwardPin;
    int PWMPin;
    int hallOnePin;
    int hallTwoPin;
    
    float newPos;
};

#endif
