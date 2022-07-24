#include "pid.h"

PID::PID(float Kp_in, float Ki_in, float Kd_in)
: Kp(Kp_in)
, Ki(Ki_in)
, Kd(Kd_in)
, my_time(millis())
, dt(0)
, last_val(0)
, last_i_val(0)
, sat_calc_val(0)
, last_comp_val(0)
{
 
}

float PID::calculate(float value, float target){
  dt = millis() - my_time;
  my_time = millis();

  float in_error = target - value;
  float dot_error = (in_error - last_val)/dt;
  float int_error = last_i_val + (in_error * dt);

  float calc_val = Kp * in_error + Ki * int_error + Kd * dot_error;
  
  last_i_val = int_error;
  last_val = calc_val;

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
  dt = millis() - my_time;
  my_time = millis();

  float in_error = target - value;
  float dot_error = (in_error - last_val)/dt;
  float int_error = last_i_val + (in_error + ((1/backVal)*(sat_calc_val - last_val))* dt);

  float calc_val = Kp * in_error + Ki * int_error + Kd * dot_error;

  sat_calc_val = constrain(calc_val, saturationMin, saturationMax);
  last_i_val = int_error;
  last_val = calc_val;
  
  return(sat_calc_val);
}

float PID::compfilter(float in_val, float alpha){
  float calcAngle = ((1-alpha)*last_comp_val)+(alpha*in_val);
  last_comp_val = calcAngle;
  
  return calcAngle;
}