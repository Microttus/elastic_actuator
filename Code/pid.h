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
 * The library will handle last values and time keeping by itself
 *
 */

#ifndef PID_h
#define PID_h

class PID
{
  public:
    PID(float Kp_in, float Ki_in, float Kd_in);
    float calculate(float value, float target);
    float calcsaturation(float value, float target, float saturationMin, float saturationMax);
    float backcalc(float value, float target, float backVal, float saturationMin, float saturationMax);
    float backnoisecalc(float value, float target, float backVal, float noiseVal, float saturationMin, float saturationMax);
    float compfilter(float in_val, float alpha = 0.002);
    //float constrain(float value, float saturationMin, float saturationMax);
  
  private:
    float _Kp;
    float _Ki;
    float _Kd;
    float _last_val;
    float _last_i_val;
    float _last_d_val;
    unsigned long _mark_time;
    float _sat_calc_val;
    float _last_comp_val;
    float _last_calc_val;
    
};

#endif 
