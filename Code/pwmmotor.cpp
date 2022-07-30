#include "pwmmotor.h"

static volatile int hall_count_one = 0;
static volatile int hall_count_two = 0;
static volatile int currentMotorDir = 0; 

pwmMotor::pwmMotor(int forwardPin_in, int backwardPin_in, int pwm_in, int hallPinOne_in, int hallPinTwo_in) 
: forwardPin(forwardPin_in)
, backwardPin(backwardPin_in)
, PWMPin(pwm_in)
, hallOnePin(hallPinOne_in)
, hallTwoPin(hallPinTwo_in)
, pass_by_rot(2650.0)
, lastMotorSpeed(0)
, saturationMax(254)
, saturationMin(0)
{
  pinMode(forwardPin_in, OUTPUT);
  pinMode(backwardPin_in, OUTPUT);
  pinMode(pwm_in, OUTPUT);

  attachInterrupt(hallOnePin,increase_hall_val_one,RISING);
  //attachInterrupt(hallTwoPin,increase_hall_val_two,CHANGE);
}

void pwmMotor::goToSpeed(int motorSpeed){
  currentMotorDir = motorSpeed/abs(motorSpeed);
  if (motorSpeed != lastMotorSpeed){
    if (motorSpeed > 0){
      if (dirFlag == 0){    // 1 means clockwise rotation, and are positive
        digitalWrite(forwardPin, HIGH);
        digitalWrite(backwardPin, LOW);
        dirFlag = 1;
      }
      analogWrite(PWMPin, motorSpeed);
    } else if (motorSpeed < 0) {
      if (dirFlag == 1) {
        digitalWrite(forwardPin,LOW);
        digitalWrite(backwardPin, HIGH);
        dirFlag = 0;
      }
    }
    
    motorSpeed = constrain(abs(motorSpeed), saturationMin, saturationMax);
    analogWrite(PWMPin,motorSpeed); 
    lastMotorSpeed = motorSpeed;
  }
  return;
}

int pwmMotor::check_rotation(){
  noInterrupts();
  int number_of_pass = hall_count_one + hall_count_two;
  interrupts();
  float number_of_ang = (number_of_pass/pass_by_rot)*360.0;
  return number_of_ang;
}

void pwmMotor::increase_hall_val_one() {
  if (currentMotorDir == 1){
    hall_count_one++;
  } else if (currentMotorDir == -1){
    hall_count_one--;
  } else {}
  return;
}

void pwmMotor::increase_hall_val_two() {
  if (currentMotorDir == 1){
    hall_count_two++;
  } else if (currentMotorDir == -1){
    hall_count_two--;
  } else {}
  return;
}

void pwmMotor::reset_hall_val(){
  hall_count_one = 0;
  hall_count_two = 0;
}

int pwmMotor::return_motor_dir(){
  return currentMotorDir;
}

void pwmMotor::stop(){
  digitalWrite(forwardPin, LOW);
  digitalWrite(backwardPin, LOW);
  analogWrite(PWMPin,0); 
}
