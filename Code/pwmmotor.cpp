#include "pwmmotor.h"

static volatile int hall_count_one = 0;
static volatile int hall_count_two = 0; 

pwmMotor::pwmMotor(int forwardPin_in, int backwardPin_in, int pwm_in, int hallPinOne_in, int hallPinTwo_in) 
: forwardPin(forwardPin_in)
, backwardPin(backwardPin_in)
, PWMPin(pwm_in)
, hallOnePin(hallPinOne_in)
, hallTwoPin(hallPinTwo_in)
, pass_by_rot(12000)
{
  pinMode(forwardPin_in, OUTPUT);
  pinMode(backwardPin_in, OUTPUT);
  pinMode(pwm_in, OUTPUT);

  attachInterrupt(hallOnePin,increase_hall_val_one,CHANGE);
  attachInterrupt(hallTwoPin,increase_hall_val_two,CHANGE);
}

void pwmMotor::goToPos(float requiredPos){
  
}

int pwmMotor::check_rotation(){
  noInterrupts();
  int number_of_pass = hall_count_one + hall_count_two;
  interrupts();
  int number_of_rot = number_of_pass/pass_by_rot;
  return number_of_rot;
}

void pwmMotor::increase_hall_val_one() {
  hall_count_one++;
}

void pwmMotor::increase_hall_val_two() {
  hall_count_two++;
}
