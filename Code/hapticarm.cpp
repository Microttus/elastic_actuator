#include "hapticarm.h"

HapticArm::HapticArm(int motorSettings[], int sensorSettings[], float PIDset[][3])
: ArmSensor_(sensorSettings[0],sensorSettings[1], sensorSettings[2], sensorSettings[3])
, MainMotor_(motorSettings[0], motorSettings[1], motorSettings[2], motorSettings[3], motorSettings[4])
, PositionPID_(PIDset[0][0], PIDset[0][1], PIDset[0][1])
, ForcePID_(PIDset[1][0], PIDset[1][1], PIDset[1][2])
, MotorPID_(PIDset[2][0], PIDset[2][1], PIDset[2][2])
, armLength(200) // in mm
, my_time(millis())
, dt(0)
, lastPosition(0)
, lastMovedSpeed(0)
, calibrationSpeed(65)
, raw_min(0)
, raw_max(0)
, switchType(0)
{ 
 
}

void HapticArm::goToPos(float requiredPos){
  // Used for msking the arm go to a spesific position based on encoder
  float currentPos = ArmSensor_.readPos();
  float currentCurrent = ArmSensor_.readForce();
  
  //int calcSpeed = PositionPID_.calculate(currentPos, requiredPos);
  int calcSpeed = PositionPID_.backcalc(currentPos, requiredPos, 1, -255, 255);
  MainMotor_.goToSpeed(calcSpeed);
  emergencyCheck();

  Serial.print(calcSpeed);
  Serial.print("   ");
  Serial.println(currentCurrent);
  return;
}

void HapticArm::goToImpedance(float requiredPos, float antiConst){
  // Use for impedance control of the haptic arm
  float currentPos = ArmSensor_.readPos();
  float currentPosMotor = MainMotor_.check_rotation();
  
  float calcError = PositionPID_.calculate(currentPos, requiredPos);

  int calcSpeed = MotorPID_.calculate(currentPosMotor,requiredPos);//MotorPID_.backcalc(currentPosMotor, calcError, antiConst, saturationLimit[0], saturationLimit[1]);

  MainMotor_.goToSpeed(calcSpeed);
  emergencyCheck();

  Serial.println(calcSpeed);
  return;
}

void HapticArm::goToAdmittance(float requiredForce, float antiConst){
  // Use  for admittance control of the haptic arm
  float currentForce = ArmSensor_.readForce();
  float currentPosMotor = MainMotor_.check_rotation();
  
  float calcError = MotorPID_.calculate(currentForce, requiredForce);

  float calcSpeed = PositionPID_.backcalc(currentPosMotor, calcError, antiConst, saturationLimit[0], saturationLimit[1]);

  MainMotor_.goToSpeed(calcSpeed);
  emergencyCheck();

  Serial.println(calcSpeed);
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
  emergencyCheck();
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

 void HapticArm::calibrateArm(){
  bool cal_flag = true;
  bool cal_dir = true;
  bool null_is_max = true;

  // Ensure arm not at negative endpoint
  MainMotor_.goToSpeed(calibrationSpeed);
  delay(2000);
  
  while (cal_flag){
    int* switchList = ArmSensor_.readSwitch();
    
    if (cal_dir == true){
      MainMotor_.goToSpeed(calibrationSpeed);
      if (switchList[0] == switchType){
        raw_max = ArmSensor_.calibrateEncoder(0,0);
        cal_dir = false;
        forward_index = 0;
      } else if (switchList[1] == switchType){
        raw_max = ArmSensor_.calibrateEncoder(0,0);
        cal_dir = false;
        null_is_max = false;
        forward_index = 1;
      } else {}
    } else if (cal_dir == false){
      MainMotor_.goToSpeed(-calibrationSpeed);
      if (switchList[0] == switchType && null_is_max == false){
        raw_min = ArmSensor_.calibrateEncoder(0,0);
        cal_flag = false;
      } else if (switchList[1] == switchType && null_is_max == true){
        raw_min = ArmSensor_.calibrateEncoder(0,0);
        cal_flag = false;
      } else {}
    } else {}
  }
  
  MainMotor_.goToSpeed(0);
  MainMotor_.reset_hall_val();
  int the_val = ArmSensor_.calibrateEncoder(raw_min, raw_max);

  // Plot the calibration values to Serial Monitor
  Serial.print("Min: ");
  Serial.print(raw_min);
  Serial.print("   Max: ");
  Serial.println(raw_max);
  
  return;
 }

void HapticArm::emergencyCheck(){
  int* switchList = ArmSensor_.readSwitch();
  int currentDir = MainMotor_.return_motor_dir();

  int back_ind = abs(forward_index - 1);

  if (switchList[forward_index] == 0 && currentDir == 1){
    emergencyBreak();
  } else if (switchList[back_ind] == 0 && currentDir == -1){
    emergencyBreak();
  }
}

void HapticArm::emergencyBreak(){
  while (true) {
    MainMotor_.stop();
    Serial.println("Motor emergency break due to limits");
    delay(1000);
  }
}


