/*
 * Author:
 * Microttus
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
#include "pid.h"

class HapticSensor{
  public:
    HapticSensor(int forcePin, int currentPin, int switchPinOne, int switchPinTwo);
    float readForce();
    float readPos();
    float readCurrent();
    int* readSwitch();

    int calibrateEncoder(int newMinVal = 0, int newMaxVal = 0);
  
  private:
    int _forceSensorPin;
    int _currentSensorPin;
    int _switchOne;
    int _switchTwo;
    int _switchList[2] = {3,3};

    int _loadcellType = 5; //The kilo rating for the force sensor used to adjust the signal
    const float _g = 9.81; //m/s^2

    float _magMinVal;
    float _magMaxVal;

    float _currentGain;

    AMS_5600 magDisk_;
    PID CurrentPID_;
    PID ForcePID_;
};

#endif 
