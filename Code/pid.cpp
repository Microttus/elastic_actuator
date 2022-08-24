#include "pid.h"

PID::PID(float Kp_in, float Ki_in, float Kd_in)
: _Kp(Kp_in)
, _Ki(Ki_in)
, _Kd(Kd_in)
, _mark_time(millis())
, _last_val(0)
, _last_i_val(0)
, _sat_calc_val(0)
, _last_comp_val(0)
{
 
}

float PID::calculate(float value, float target){
  unsigned long dt = millis() - _mark_time;
  _mark_time = millis();

  float in_error = target - value;
  float int_error = _last_i_val + (in_error * dt);
  float dot_error = (in_error - _last_val)/dt;

  float calc_val = _Kp * in_error + _Ki * int_error + _Kd * dot_error;
  
  _last_i_val = int_error;
  _last_val = calc_val;

  /*
  Serial.print("dt: ");
  Serial.print(dt);
  Serial.print("   in_error: ");
  Serial.print(in_error);
  Serial.print("   dot_error: ");
  Serial.print(dot_error);
  Serial.print("   int_error: ");
  Serial.println(int_error);
  */

  return(calc_val);
}

float PID::backcalc(float value, float target, float backVal, float saturationMin, float saturationMax){
  unsigned long dt = millis() - _mark_time;
  _mark_time = millis();

  float in_error = target - value;
  float int_error = _last_i_val + (in_error + ((1/backVal)*(_sat_calc_val - _last_val))* dt);
  float dot_error = (in_error - _last_val)/dt;

  float calc_val = _Kp * in_error + _Ki * int_error + _Kd * dot_error;

  _sat_calc_val = constrain(calc_val, saturationMin, saturationMax);
  _last_i_val = int_error;
  _last_val = calc_val;
  
  Serial.print("dt: ");
  Serial.print(dt);
  Serial.print("   in_error: ");
  Serial.print(in_error);
  Serial.print("   dot_error: ");
  Serial.print(dot_error);
  Serial.print("   int_error: ");
  Serial.print(int_error);
  
  return(_sat_calc_val);
}

float PID::compfilter(float in_val, float alpha){
  float calcAngle = ((1-alpha)*_last_comp_val)+(alpha*in_val);
  _last_comp_val = calcAngle;
  
  return calcAngle;
}