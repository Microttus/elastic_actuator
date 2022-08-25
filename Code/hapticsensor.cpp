#include "hapticsensor.h"

HapticSensor::HapticSensor(int forcePin, int currentPin, int switchPinOne, int switchPinTwo)
: _forceSensorPin(forcePin)
, _currentSensorPin(currentPin)
, _magMinVal(511)
, _magMaxVal(3354)
, _currentGain(800)
, _switchOne(switchPinOne)
, _switchTwo(switchPinTwo)
, CurrentPID_(0,0,0)
, ForcePID_(0,0,0)
{
  analogReadResolution(10);
  pinMode(_switchOne, INPUT);
  pinMode(_switchTwo, INPUT);
  //digitalWrite(_switchOne, LOW);
  //digitalWrite(_switchTwo, LOW);
  pinMode(_forceSensorPin, INPUT);
  pinMode(_currentSensorPin, INPUT);

  accgyr_.initialize();
}

float HapticSensor::readForce(){
  // Need prof ceck for how the load cell are to be read
  float loadcellVoltage = analogRead(_forceSensorPin);

  // Adjust for angle of arm
  float addAng = readPos();
  float addVolt = 497 + 0.26983*addAng - 0.0045427*pow(addAng,2) + 0.000012153*pow(addAng,3);
  float adjVolt = loadcellVoltage - addVolt;

  //float calc_weight = 9000-(18.453*adjVolt);
  float calc_weight = 4 + (11.25*adjVolt);

  // Nedd adjustment for angle
  float messuredForce = (calc_weight/1000) * _g;
  float load_com = ForcePID_.compfilter(messuredForce,0.02);
  
  return load_com;
}

float HapticSensor::readPos(){
  float out_max = 250.0;
  float out_min = 0.0;
  float raw_val = magDisk_.getRawAngle();
  float angle_pos = ((raw_val-_magMinVal) * (out_max-out_min)) / ((_magMaxVal - _magMinVal)) + out_min; 
  return angle_pos;
}

float HapticSensor::readCurrent(){
  int raw_val = analogRead(_currentSensorPin);
  float raw_volt = ((raw_val * 5.0)/1023)-2.487;
  float current_read = (raw_volt*_currentGain);
  float current_comp = CurrentPID_.compfilter(current_read,0.02);
  return current_comp;
}

int* HapticSensor::readSwitch(){
  _switchList[0] = digitalRead(_switchOne);
  _switchList[1] = digitalRead(_switchTwo);
  return _switchList;
}

int HapticSensor::calibrateEncoder(int newMinVal, int newMaxVal){
  _magMinVal = newMinVal;
  _magMaxVal = newMaxVal;

  int raw_val = magDisk_.getRawAngle();
  
  return raw_val;
}

void HapticSensor::updateAccGyr( int16_t& acx,  int16_t& acy,  int16_t& acz, int16_t& gyx, int16_t& gyy, int16_t& gyz){
  accgyr_.getMotion6(&acx, &acy, &acz, &gyx, &gyy, &gyz);
}