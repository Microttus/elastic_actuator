#include "hapticsensor.h"

HapticSensor::HapticSensor(int forcePin, int currentPin, int switchPinOne, int switchPinTwo)
: forceSensorPin(forcePin)
, currentSensorPin(currentPin)
, magMinVal(767)
, magMaxVal(3617)
, currentGain(400)
, switchOne(switchPinOne)
, switchTwo(switchPinTwo)
, CurrentPID_(0,0,0)
, ForcePID_(0,0,0)
{
  analogReadResolution(10);
  pinMode(switchOne, INPUT);
  pinMode(switchTwo, INPUT);
  digitalWrite(switchOne, HIGH);
  digitalWrite(switchTwo, HIGH);
  pinMode(forceSensorPin, INPUT);
  pinMode(currentSensorPin, INPUT);
}

float HapticSensor::readForce(){
  // Need prof ceck for how the load cell are to be read
  float loadcellVoltage = analogRead(forceSensorPin);

  // Adjust for angle of arm
  float addAng = readPos();
  float addVolt = 14 -2.3283*addAng + 0.077665*pow(addAng,2) - 0.00090024*pow(addAng,3) + 0.0000042154*pow(addAng,4) - 0.00000000686*pow(addAng,5);
  float adjVolt = loadcellVoltage - addVolt;

  float calc_weight = 9000-(18.453*adjVolt);
  // Nedd adjustment for angle
  float messuredForce = (calc_weight/1000) * g;
  float force_comp = ForcePID_.compfilter(messuredForce,0.1);
  
  return force_comp;
}

float HapticSensor::readPos(){
  int raw_val = magDisk_.getRawAngle();
  float angle_pos = map(raw_val, magMinVal, magMaxVal, 0, 250);
  
  return angle_pos;
}

float HapticSensor::readCurrent(){
  int raw_val = analogRead(currentSensorPin);
  float raw_volt = ((raw_val * 5.0)/1023)-2.5125;
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
