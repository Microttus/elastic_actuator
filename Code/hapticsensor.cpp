#include "hapticsensor.h"

HapticSensor::HapticSensor(int forcePin, int currentPin)
: forceSensorPin(forcePin)
, currentSensorPin(currentPin)
, magMinVal(0)
, magMaxVal(4096)
, currentGain(400)
{
 
}

float HapticSensor::readForce(){
  float loadcellVoltage = analogRead(forceSensorPin);
  float messuredWeight = ((loadcellVoltage/2.5)-2.5)*loadcellType; 
  float messuredForce = messuredWeight * g;
  
  return messuredForce;
}

float HapticSensor::readPos(){
  int raw_val = magDisk_.getRawAngle();
  float angle_pos = map(raw_val, magMinVal, magMaxVal, 0, 180);

  return angle_pos;
}

float HapticSensor::readCurrent(){
  int raw_val = analogRead(currentSensorPin);
  float raw_volt = ((raw_val * 5)/1023) - 2500;
  float current_read = raw_volt/currentGain;

  return currentGain;
}
