#ifndef pwmmotor_h
#define pwmmotor_h

#include "Arduino.h"

class pwmMotor{
  public:
    pwmMotor(int forwardPin_in, int backwardPin_in, int pwm_in, int hallPinOne_in, int hallPinTwo_in);
    void goToPos(float requiredPos);
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
    
    float newPos;
};

#endif
