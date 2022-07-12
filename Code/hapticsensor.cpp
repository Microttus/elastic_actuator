#include "hapticsensor.h"

HapticSensor::HapticSensor(int forcePin, int currentPin)
: forceSensorPin(forcePin)
, currentSensorPin(currentPin)
, magMinVal(835)
, magMaxVal(3676)
, currentGain(400)
{
  analogReadResolution(10);
}

float HapticSensor::readForce(){
  // Need prof ceck for how the load cell are to be read
  float loadcellVoltage = analogRead(forceSensorPin);
  float messuredWeight = (((loadcellVoltage*2)/1023)-1)*loadcellType; 
  float messuredForce = messuredWeight * g;
  
  return messuredWeight;
}

float HapticSensor::readPos(){
  int raw_val = magDisk_.getRawAngle();
  float angle_pos = map(raw_val, magMinVal, magMaxVal, 0, 250);
  
  return angle_pos;
}

float HapticSensor::readCurrent(){
  int raw_val = analogRead(currentSensorPin);
  float raw_volt = ((raw_val * 5.0)/1023);
  float current_read = raw_volt/(currentGain*0.001);
  
  return currentGain;
}

float HapticSensor::readSwitch(){
  
}

float HapticSensor::calibrateEncoder(float newMinVal, float newMaxVal){
  magMinVal = newMinVal;
  magMaxVal = newMaxVal;
}
