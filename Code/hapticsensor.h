/*
 * Author:
 * Maritn Økter
 * 
 * 
 * 
 * This liabry requires: arduino and AS5600
 */

#ifndef HapticSensor_h
#define HapticSensor_h

#include "Arduino.h"
#include "AS5600.h"

class HapticSensor{
  public:
    HapticSensor(int forcePin, int currentPin);
    float readForce();
    float readPos();
  
  private:
    int forceSensorPin;
    int currentSensorPin;

    int loadcellType = 20; //The kilo rating for the force sensor used to adjust the signal
    float g = 9.81; //m/s^2

    int magMinVal;
    int magMaxVal;

    AMS_5600 magDisk_;
};

#endif 
