/*
 * Author:
 * Maritn Økter
 * 
 * This libary is a cloection of funktions for correct readings 
 * of the different integrated sensores in the Haptic Arm
 * 
 * Use readForce() for the forcesensor, but remember to adjust the 
 * instrument amplifier for zeroing
 * 
 * Use the readPos() for reading the position encoder. This 
 * function use the libary AS5600 for reading
 * 
 * Use readCurrent() for reading the current fed to the motor,
 * remember to adjust the gain setting if a different current 
 * sensor is used
 * 
 * This liabry requires: arduino and AS5600 and Wiere
 */

#ifndef HapticSensor_h
#define HapticSensor_h

#include "Arduino.h"
#include "AS5600.h"
#include "Wire.h"

class HapticSensor{
  public:
    HapticSensor(int forcePin, int currentPin);
    float readForce();
    float readPos();
    float readCurrent();
    float readSwitch();
    float calibrateEncoder(float newMinVal, float newMaxVal);
  
  private:
    int forceSensorPin;
    int currentSensorPin;

    int loadcellType = 20; //The kilo rating for the force sensor used to adjust the signal
    float g = 9.81; //m/s^2

    int magMinVal;
    int magMaxVal;

    float currentGain;

    AMS_5600 magDisk_;
};

#endif 
