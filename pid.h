/*
 * Author:
 * Microttus
 *  
 * A short libary for impolimenting a PID controller into the system 
 * 
 * Include
 * - Normal PID
 * - PID with backcalculation saturation
 * 
 * Initialize the PID with the wanted Kp, Ki and Kd values for the wanted PID
 * Use calculate(current value, desired value) to calculate the next step for the controller 
 * 
 * The libary will handle last values and time keeping by itself
 * 
 * This liabry requires: Arduino
 */

#ifndef PID_h
#define PID_h

#include "Arduino.h"

class PID
{
  public:
    PID(float Kp_in, float Ki_in, float Kd_in);
    float calculate(float value, float target);
    float backcalc(float value, float target, float backVal, float saturationMin, float saturationMax);
    float compfilter(float in_val, float alpha = 0.002);
  
  private:
    float _Kp;
    float _Ki;
    float _Kd;
    float _last_val;
    float _last_i_val;
    unsigned long _mark_time;
    float _sat_calc_val;
    float _last_comp_val;
    
};

#endif 
