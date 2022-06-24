#include "pwmmotor.h"

static volatile int hall_count_one = 0;
static volatile int hall_count_two = 0; 

pwmMotor::pwmMotor(int forwardPin_in, int backwardPin_in, int pwm_in, int hallPinOne_in, int hallPinTwo_in) 
: forwardPin(forwardPin_in)
, backwardPin(backwardPin_in)
, PWMPin(pwm_in)
, hallOnePin(hallPinOne_in)
, hallTwoPin(hallPinTwo_in)
{
  pinMode(forwardPin_in, OUTPUT);
  pinMode(backwardPin_in, OUTPUT);
  pinMode(pwm_in, OUTPUT);

  attachInterrupt(hallOnePin,increase_hall_vall_one,CHANGE);
}

void pwmMotor::goToPos(float requiredPos){
  
}

void pwmMotor::increase_hall_vall_one() {
  hall_count_one++;
}
