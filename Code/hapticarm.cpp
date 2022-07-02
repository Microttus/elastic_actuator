#include "hapticarm.h"

HapticArm::HapticArm(int motorSettings[], int sensorSettings[], int PosSet[], int ForceSet[])
: ArmSensor_(sensorSettings[0],sensorSettings[1])
, MainMotor_(motorSettings[0], motorSettings[1], motorSettings[2], motorSettings[3], motorSettings[4])
, PositionPID_(PosSet[0], PosSet[1], PosSet[2])
, ForcePID_(ForceSet[0], ForceSet[1], ForceSet[2])
{  
}

void HapticArm::goToPos(float requiredPos){
  // Need to implement angle sensor
  //Hente data fra sensorer
  float currentPos = ArmSensor_.readPos();
  
  int calcSpeed = PositionPID_.calculate(currentPos, requiredPos);
  MainMotor_.goToSpeed(calcSpeed);
  return;
}

void HapticArm::resistForce(float forceThreshold){
  // Used for resisting a force in 
  int calcSpeed; // Initialise for use
  float currentForce = ArmSensor_.readForce();

  if (currentForce >= forceThreshold){
    float currentPos = ArmSensor_.readPos();
    calcSpeed = PositionPID_.calculate(currentPos, 0);
  } else {
    calcSpeed = ForcePID_.calculate(currentForce, forceThreshold);
  }
  MainMotor_.goToSpeed(calcSpeed);
  return;
}
