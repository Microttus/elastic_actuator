/*
 * Author:
 * Microttus
 *
 * A short example for how to use the magnetic encoder
 *
 * Include:
 * - AS5600.h
 *
 * Connect diagram:
 * SDA  -->  A4
 * SCL  -->  A5
 * 5V   -->  5V XIAO
 * GND  -->  GND XIAO
 *
 * NOTE:
 * Save this file in a folder with same name. 
 * In the same folder add AS5600.h and AS5600.cpp
 * Files avaiable at github.com/Microttus/HapticSommerSchool
 *
 * OBS!:
 * If values is to high and goes to zero in the operational angles rotate sensor chip 90 degrees
 */

 #include "AS5600.h"
 #include "Wire.h"

// Initialize encoder object
 AMS_5600 magDisk_; // Can be named what you like (_ at the end of the name is good naming policy)

void setup() {
  Wire.begin();
  Serial.begin(9600);  // Set up Serial Monitor
}

void loop() {
  // Read encoder
  int raw_val = magDisk_.getRawAngle();

  // Print raw_val to Serial monitor
  Serial.print("Raw value from sensor:  ");
  Serial.println(raw_val);

  // Calculation for angles
  int minMagValAtZeroDeg = 1000;  // Read from Serial monitor!
  int maxMagValAtZeroDeg = 3000;  // Read from Serial monitor!

  int maxAngleOfArm = 250;        // From design

  int angle_val = map(raw_val, minMagValAtZeroDeg, maxMagValAtZeroDeg, 0, maxAngleOfArm);

  // Print Angles
  // Serial.print("Calculated angle of arm:  ")
  // Serial.println(angle_val)

}
