#include "hapticsensor.h"

HapticSensor::HapticSensor(int forcePin, int currentPin, int switchPinOne, int switchPinTwo)
: forceSensorPin(forcePin)
, currentSensorPin(currentPin)
, magMinVal(1261)
, magMaxVal(3820)
, currentGain(400)
, switchOne(switchPinOne)
, switchTwo(switchPinTwo)
, CurrentPID_(0,0,0)
, ForcePID_(0,0,0)
{
  analogReadResolution(10);
  pinMode(switchOne, INPUT);
  pinMode(switchTwo, INPUT);
  pinMode(forceSensorPin, INPUT);
  pinMode(currentSensorPin, INPUT);
}

float HapticSensor::readForce(){
  // Need prof ceck for how the load cell are to be read
  float loadcellVoltage = analogRead(forceSensorPin);
  float messuredWeight = loadcellVoltage*loadcellType; 
  float messuredForce = messuredWeight * g;
  float calc_weight = 9000-(18.453*loadcellVoltage);
  float force_comp = ForcePID_.compfilter(calc_weight);
  
  return force_comp;
}

float HapticSensor::readPos(){
  int raw_val = magDisk_.getRawAngle();
  float angle_pos = map(raw_val, magMinVal, magMaxVal, 0, 250);
  
  return angle_pos;
}

float HapticSensor::readCurrent(){
  int raw_val = analogRead(currentSensorPin);
  float raw_volt = ((raw_val * 5.0)/1023)-2.5175;
  float current_read = (raw_volt*currentGain);
  float current_comp = CurrentPID_.compfilter(current_read);
  return current_comp;
}

int* HapticSensor::readSwitch(){
  switchList[0] = digitalRead(switchOne);
  switchList[1] = digitalRead(switchTwo);
  return switchList;
}

int HapticSensor::calibrateEncoder(int newMinVal, int newMaxVal){
  magMinVal = newMinVal;
  magMaxVal = newMaxVal;

  int raw_val = magDisk_.getRawAngle();
  
  return raw_val;
}
