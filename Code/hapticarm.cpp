#include "hapticarm.h"

HapticArm::HapticArm(int motorSettings[], int sensorSettings[], float PIDset[][3])
: ArmSensor_(sensorSettings[0],sensorSettings[1], sensorSettings[2], sensorSettings[3])
, MainMotor_(motorSettings[0], motorSettings[1], motorSettings[2], motorSettings[3], motorSettings[4])
, PositionPID_(PIDset[0][0], PIDset[0][1], PIDset[0][1])
, ForcePID_(PIDset[1][0], PIDset[1][1], PIDset[1][2])
, MotorPID_(PIDset[2][0], PIDset[2][1], PIDset[2][2])
, _armLength(200) // in mm
, _mark_time(millis())
, _lastPosition(0)
, _lastMovedSpeed(0)
, _lastAngle(0)
, _lastMovedAngleSpeed(0)
, _calibrationSpeed(100)
, _switchType(1)  // NO = 1; NC = 0;
, _currentAngAd(30)
, _currentTorqueImp(0)
{ 
 
}

void HapticArm::goToPos(float requiredPos){
  // Used for making the arm go to a spesific position based on encoder

  // Retriving sensor data
  float currentPos = ArmSensor_.readPos();
  float currentForce = ArmSensor_.readForce();
  
  // Calculate the desired speed with a PID controller and set the motor speed
  int calcSpeed = PositionPID_.backcalc(currentPos, requiredPos, 1, -255, 255);
  MainMotor_.goToSpeed(calcSpeed);
  
  // Print relevant data to the serial monitor
  
  Serial.print("   ");
  Serial.print(calcSpeed);
  Serial.print("   ");
  Serial.print(currentPos);
  Serial.print("   ");
  Serial.println(currentForce);
  
  // Emergency check and return
  emergencyCheck();
  return;
}

float HapticArm::goImpedance(float massConstant, float damperConstant, float springConstant, float initialPosition){
  // Use for impedance control of the haptic arm

  // Retrive snesor data
  movedAngle(); // Update the moved angle value of the arm
  float currentCurrent = ArmSensor_.readCurrent();
  float currentForce = ArmSensor_.readForce();
  float currentPos = ArmSensor_.readPos();
  
  // Calculate the desired torque with equation 
  float calcTorque = (massConstant*_AngVelAks[2]) + (damperConstant*_AngVelAks[1]) + (springConstant*_AngVelAks[0]);
  float messuredTorque = currentForce * (_armLength/1000);     // For comparison

  // Calculate the desired speed with a PID controller and set the motor speed 
  int calcSpeed = ForcePID_.backcalc(currentCurrent, calcTorque, 1, -255, 255);
  MainMotor_.goToSpeed(calcSpeed);
  
  // Print relevant data to the serial monitor
  Serial.print("  Impedance control: "); 
  Serial.print(calcSpeed);
  Serial.print("   ");
  Serial.print(calcTorque);
  Serial.print("   ");
  Serial.print(currentCurrent);
  Serial.println("   ");

  // Emergency check and return
  emergencyCheck();
  return currentPos;
}

 float HapticArm::goAdmittance(float massConstant, float damperConstant, float springConstant, float initialPosition, float initialForce){
  // Used for the arm to act as a spring damper system

  // Retrive sensor data
  movedLength(); // Update the moved length of the arm
  float currentForce = ArmSensor_.readForce();
  float currentPos = ArmSensor_.readPos();

  // Calculate the desired position with equation
  float newPos = (currentForce - (damperConstant*_PosVelAks[1]) - (massConstant*_PosVelAks[2]))/springConstant;


  // Calculate the adiason angle
  float newAng = atan2(newPos,_armLength) * RAD_TO_DEG;

  // Control in set zero position mode or in "teach" mode
  if (initialPosition == -1){
    float controlPos = - newAng + _currentAngAd;
    _currentAngAd = constrain(controlPos, 0, 250); // Use if "teach robota arm" mode is desired
  } else {
    _currentAngAd = - newAng + 125; // Use if center around center is desired
  }
  
  // Print relevant data to the serial monitor
  
  Serial.print(newAng);
  Serial.print("   ");
  Serial.print(currentForce);
  Serial.print("   ");
  Serial.print(_currentAngAd);
  Serial.print("   ");
  
  // Direct the new desired position to the position method for PID control and return
  goToPos(_currentAngAd);
  return currentPos;  
 }

void HapticArm::movedLength(){
  unsigned long dt = millis() - _mark_time;
  _mark_time = millis();

  float currentPos = ArmSensor_.readPos();
  float movedLength = _armLength * tan((currentPos-_lastPosition)*DEG_TO_RAD);

  float movedSpeed = movedLength/dt;
  float movedAks = (movedSpeed - _lastMovedSpeed)/dt;
  
  _lastPosition = currentPos;
  _lastMovedSpeed = movedSpeed;

  _PosVelAks[0] = movedLength;
  _PosVelAks[1] = movedSpeed;
  _PosVelAks[2] = movedAks;

  return;  
}

void HapticArm::movedAngle(){
  unsigned long dt = millis() - _mark_time;
  _mark_time = millis();

  float currentPos = ArmSensor_.readPos();
  float movedAngle = currentPos - _lastAngle;

  float movedSpeed = movedAngle/dt;
  float movedAks = (movedSpeed - _lastMovedAngleSpeed)/dt;

  _lastAngle = currentPos;
  _lastMovedAngleSpeed = movedSpeed;

  _AngVelAks[0] = movedAngle;
  _AngVelAks[1] = movedSpeed;
  _AngVelAks[2] = movedAks;

  return;  
}

void HapticArm::calibrateArm(){
  bool cal_flag = true;
  bool cal_dir = true;
  bool null_is_max = true;
  int raw_min  = 0;
  int raw_max = 0;

  // Ensure arm not at negative endpoint
  Serial.println("Calibration started");
  MainMotor_.goToSpeed(_calibrationSpeed);
  delay(2000);
  
  while (cal_flag){
    int* _switchList = ArmSensor_.readSwitch();
    if (cal_dir == true){
      MainMotor_.goToSpeed(_calibrationSpeed);
      if (_switchList[0] == _switchType){
        raw_max = ArmSensor_.calibrateEncoder();
        cal_dir = false;
        _forward_index = 0;
      } else if (_switchList[1] == _switchType){
        raw_max = ArmSensor_.calibrateEncoder();
        cal_dir = false;
        null_is_max = false;
        _forward_index = 1;
      } else {}
    } else if (cal_dir == false){
      MainMotor_.goToSpeed(-_calibrationSpeed);
      if (_switchList[0] == _switchType && null_is_max == false){
        raw_min = ArmSensor_.calibrateEncoder();
        cal_flag = false;
      } else if (_switchList[1] == _switchType && null_is_max == true){
        raw_min = ArmSensor_.calibrateEncoder();
        cal_flag = false;
      } else {}
    } else {}
  }
  Serial.print(cal_flag);
  
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

  int back_ind = abs(_forward_index - 1);

  if (switchList[_forward_index] == _switchType && currentDir == 1){
    emergencyBreak();
  } else if (switchList[back_ind] == _switchType && currentDir == -1){
    emergencyBreak();
  }
}

void HapticArm::emergencyBreak(){
  while (true) {
    MainMotor_.stop();
    Serial.println("Motor emergency break due to end limits");
    delay(1000);
  }
}

void HapticArm::studentProgram(){
  
  
  return;
}

