#include "hapticarm.h"

HapticArm::HapticArm(int motorSettings[], int sensorSettings[], float PosSet[], float ForceSet[])
: ArmSensor_(sensorSettings[0],sensorSettings[1])
, MainMotor_(motorSettings[0], motorSettings[1], motorSettings[2], motorSettings[3], motorSettings[4])
, PositionPID_(PosSet[0], PosSet[1], PosSet[2])
, ForcePID_(ForceSet[0], ForceSet[1], ForceSet[2])
, armLength(200) // in mm
, my_time(millis())
, dt(0)
, lastPosition(0)
, lastMovedSpeed(0)
{ 
  //Serial.println(" Arm made");
}

void HapticArm::goToPos(float requiredPos){
  // Used for msking the arm go to a spesific position based on encoder
  float currentPos = ArmSensor_.readPos();
  
  int calcSpeed = PositionPID_.calculate(currentPos, requiredPos);
  MainMotor_.goToSpeed(calcSpeed);

  float currentCurrent = ArmSensor_.readForce();
  Serial.println(currentPos);
  return;
}

void HapticArm::resistForce(float forceThreshold){
  // Used for resisting a force in 
  int calcSpeed; // Initialise for use
  float currentForce = ArmSensor_.readForce();
  //Serial.println(currentForce);

  if (currentForce <= forceThreshold){
    float currentPos = ArmSensor_.readPos();
    calcSpeed = PositionPID_.calculate(currentPos, 90);
  } else {
    calcSpeed = ForcePID_.calculate(currentForce, forceThreshold);
  }
  Serial.println(calcSpeed);
  MainMotor_.goToSpeed(calcSpeed);
  return;
}

float* HapticArm::calcMovement(){
  dt = millis() - my_time;
  my_time = millis();

  float currentPos = ArmSensor_.readPos();
  float movedLength = armLength * tan(currentPos - lastPosition);
  float movedSpeed = movedLength/dt;
  float movedAks = (movedSpeed - lastMovedSpeed)/dt;

  lastPosition = currentPos;
  lastMovedSpeed = movedSpeed;

  float returnVals[3] = {movedLength, movedSpeed, movedAks};

  return returnVals;  
}

 void HapticArm::goSpring(float massConstant, float damperConstant, float springConstant, float initialPosition){
  // Used for the arm to act as a spring damper system
  float currentForce = ArmSensor_.readForce();

  float* movedVal = calcMovement();

  float newPos = (currentForce - (damperConstant*movedVal[1]) - (massConstant*movedVal[2]))/springConstant;
  float controlPos = newPos + initialPosition;

  goToPos(controlPos);

  return;  
 }

 void calibrateArm(){
  
 }
