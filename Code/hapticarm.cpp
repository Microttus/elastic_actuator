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
, lastAngle(0)
, lastMovedAngleSpeed(0)
, calibrationSpeed(65)
, raw_min(0)
, raw_max(0)
, switchType(1)
{ 
 
}

void HapticArm::goToPos(float requiredPos){
  // Used for msking the arm go to a spesific position based on encoder
  float currentCurrent = ArmSensor_.readCurrent();
  float currentPos = ArmSensor_.readPos();
  float currentForce = ArmSensor_.readForce();
  
  //int calcSpeed = PositionPID_.calculate(currentPos, requiredPos);
  int calcSpeed = PositionPID_.backcalc(currentPos, requiredPos, 1, -255, 255);
  MainMotor_.goToSpeed(calcSpeed);
  //emergencyCheck();

  Serial.print(calcSpeed);
  Serial.print("   ");
  Serial.print(currentPos);
  Serial.print("   ");
  Serial.print(currentCurrent);
  Serial.print("   ");
  Serial.println(currentForce);
  return;
}

void HapticArm::goImpedance(float massConstant, float damperConstant, float springConstant){
  // Use for impedance control of the haptic arm
  float* movedVal = movedAngle();
  float currentCurrent = ArmSensor_.readCurrent();
  
  float calcTourque = (massConstant*movedVal[2]) + (damperConstant*movedVal[1]) + (springConstant*movedVal[2]);

  int calcSpeed = ForcePID_.calculate(currentCurrent, calcTourque);

  MainMotor_.goToSpeed(calcSpeed);
  emergencyCheck();

  Serial.println(calcSpeed);
  return;
}

 void HapticArm::goAdmittance(float massConstant, float damperConstant, float springConstant, float initialPosition){
  // Used for the arm to act as a spring damper system
  float* movedVal = movedLength();
  float currentForce = ArmSensor_.readForce();

  float newPos = (currentForce - (damperConstant*movedVal[1]) - (massConstant*movedVal[2]))/springConstant;

  // Calculate the adiason angle
  float newAng = atan(newPos/armLength);
  float controlPos = newAng + initialPosition;

  Serial.print(newAng);
  Serial.print("   ");
  Serial.print(currentForce);
  Serial.print("   ");

  goToPos(controlPos);

  return;  
 }

float* HapticArm::movedLength(){
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

float* HapticArm::movedAngle(){
  dt = millis() - my_time;
  my_time = millis();

  float currentPos = ArmSensor_.readPos();
  float movedAngle = currentPos - lastPosition;

  float movedSpeed = movedAngle/dt;
  float movedAks = (movedSpeed - lastMovedAngleSpeed)/dt;

  lastAngle = currentPos;
  lastMovedAngleSpeed = movedSpeed;

  float returnVals[3] = {movedAngle, movedSpeed, movedAks};

  return returnVals;  
}

void HapticArm::calibrateArm(){
  bool cal_flag = true;
  bool cal_dir = true;
  bool null_is_max = true;

  // Ensure arm not at negative endpoint
  Serial.println("Cal started");
  MainMotor_.goToSpeed(calibrationSpeed);
  delay(2000);
  
  while (cal_flag){
    int* switchList = ArmSensor_.readSwitch();

    if (cal_dir == true){
      MainMotor_.goToSpeed(calibrationSpeed);
      if (switchList[0] == switchType){
        raw_max = ArmSensor_.calibrateEncoder();
        cal_dir = false;
        forward_index = 0;
      } else if (switchList[1] == switchType){
        raw_max = ArmSensor_.calibrateEncoder();
        cal_dir = false;
        null_is_max = false;
        forward_index = 1;
      } else {}
    } else if (cal_dir == false){
      MainMotor_.goToSpeed(-calibrationSpeed);
      if (switchList[0] == switchType && null_is_max == false){
        raw_min = ArmSensor_.calibrateEncoder();
        cal_flag = false;
      } else if (switchList[1] == switchType && null_is_max == true){
        raw_min = ArmSensor_.calibrateEncoder();
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

  if (switchList[forward_index] == 1 && currentDir == 1){
    emergencyBreak();
  } else if (switchList[back_ind] == 1 && currentDir == -1){
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


