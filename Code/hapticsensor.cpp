#include "hapticsensor.h"

HapticSensor::HapticSensor(int forcePin, int currentPin)
: forceSensorPin(forcePin)
, currentSensorPin(currentPin)
, magMinVal(0)
, magMaxVal(4096)
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
