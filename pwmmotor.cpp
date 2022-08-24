#include "pwmmotor.h"

static volatile int _hall_count_one = 0;
static volatile int _hall_count_two = 0;
static volatile int _currentMotorDir = 0; 

pwmMotor::pwmMotor(int forwardPin_in, int backwardPin_in, int pwm_in, int hallPinOne_in, int hallPinTwo_in) 
: _forwardPin(forwardPin_in)
, _backwardPin(backwardPin_in)
, _PWMPin(pwm_in)
, _hallOnePin(hallPinOne_in)
, _hallTwoPin(hallPinTwo_in)
, _pass_by_rot(2650.0)
, _lastMotorSpeed(0)
, _saturationMax(254)
, _saturationMin(0)
{
  pinMode(forwardPin_in, OUTPUT);
  pinMode(backwardPin_in, OUTPUT);
  pinMode(pwm_in, OUTPUT);

  attachInterrupt(_hallOnePin,increase_hall_val_one,RISING);
  //attachInterrupt(hallTwoPin,increase_hall_val_two,CHANGE);
}

void pwmMotor::goToSpeed(int motorSpeed){
  _currentMotorDir = motorSpeed/abs(motorSpeed);
  if (motorSpeed != _lastMotorSpeed){
    if (motorSpeed > 0){
      if (_dirFlag == 0){    // 1 means clockwise rotation, and are positive
        digitalWrite(_forwardPin, HIGH);
        digitalWrite(_backwardPin, LOW);
        _dirFlag = 1;
      }
      analogWrite(_PWMPin, motorSpeed);
    } else if (motorSpeed < 0) {
      if (_dirFlag == 1) {
        digitalWrite(_forwardPin,LOW);
        digitalWrite(_backwardPin, HIGH);
        _dirFlag = 0;
      }
    }
    
    motorSpeed = constrain(abs(motorSpeed), _saturationMin, _saturationMax);
    analogWrite(_PWMPin,motorSpeed); 
    _lastMotorSpeed = motorSpeed;
  }
  return;
}

int pwmMotor::check_rotation(){
  noInterrupts();
  int number_of_pass = _hall_count_one + _hall_count_two;
  interrupts();
  float number_of_ang = (number_of_pass/_pass_by_rot)*360.0;
  return number_of_ang;
}

void pwmMotor::increase_hall_val_one() {
  if (_currentMotorDir == 1){
    _hall_count_one++;
  } else if (_currentMotorDir == -1){
    _hall_count_one--;
  } else {}
  return;
}

void pwmMotor::increase_hall_val_two() {
  if (_currentMotorDir == 1){
    _hall_count_two++;
  } else if (_currentMotorDir == -1){
    _hall_count_two--;
  } else {}
  return;
}

void pwmMotor::reset_hall_val(){
  _hall_count_one = 0;
  _hall_count_two = 0;
}

int pwmMotor::return_motor_dir(){
  return _currentMotorDir;
}

void pwmMotor::stop(){
  digitalWrite(_forwardPin, LOW);
  digitalWrite(_backwardPin, LOW);
  analogWrite(_PWMPin,0); 
}
