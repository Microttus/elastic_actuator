#include "hapticsensor.h"

HapticSensor::HapticSensor(int forcePin, int currentPin, int switchPinOne, int switchPinTwo)
: forceSensorPin(forcePin)
, currentSensorPin(currentPin)
, magMinVal(511)
, magMaxVal(3354)
, currentGain(800)
, switchOne(switchPinOne)
, switchTwo(switchPinTwo)
, CurrentPID_(0,0,0)
, ForcePID_(0,0,0)
{
  analogReadResolution(10);
  pinMode(switchOne, INPUT);
  pinMode(switchTwo, INPUT);
  //digitalWrite(switchOne, LOW);
  //digitalWrite(switchTwo, LOW);
  pinMode(forceSensorPin, INPUT);
  pinMode(currentSensorPin, INPUT);
}

float HapticSensor::readForce(){
  // Need prof ceck for how the load cell are to be read
  float loadcellVoltage = analogRead(forceSensorPin);

  // Adjust for angle of arm
  float addAng = readPos();
  float addVolt = 497 + 0.26983*addAng - 0.0045427*pow(addAng,2) + 0.000012153*pow(addAng,3);
  float adjVolt = loadcellVoltage - addVolt;

  //float calc_weight = 9000-(18.453*adjVolt);
  float calc_weight = 4 + (11.25*adjVolt);

  // Nedd adjustment for angle
  float messuredForce = (calc_weight/1000) * g;
  float load_com = ForcePID_.compfilter(messuredForce,0.02);
  
  return load_com;
}

float HapticSensor::readPos(){
  int raw_val = magDisk_.getRawAngle();
  float angle_pos = map(raw_val, magMinVal, magMaxVal, 0, 250);
  
  return angle_pos;
}

float HapticSensor::readCurrent(){
  int raw_val = analogRead(currentSensorPin);
  float raw_volt = ((raw_val * 5.0)/1023)-2.487;
  float current_read = (raw_volt*currentGain);
  float current_comp = CurrentPID_.compfilter(current_read,0.02);
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
