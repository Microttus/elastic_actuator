# Elastic Arm

This device was developed and tested during the Haptic Summer School
at UiA for the EMERALD project Autum 2022.


*If you are to publish work base don this project please cite:*

```text
@article{sanfilippo2024open,
  title={Open-source design of low-cost sensorised elastic actuators for collaborative prosthetics and orthotics},
  author={Sanfilippo, Filippo and {\O}kter, Martin and Dale, J{\o}rgen and Tuan, Hua Minh and Zafar, Muhammad Hamza and Ottestad, Morten},
  journal={HardwareX},
  volume={19},
  pages={e00564},
  year={2024},
  publisher={Elsevier}
}
```

## Table of Content

1. [Motivation](#motivation)
1. [Demonstration](#Video)
2. [Intro](#the-haptic-arm-lab)
3. [Code](#code)
4. [CAD](#cad)
5. [How To Use Seeeduino](#implementation-of-seeeduino-in-arduino-ide)
6. [How To Use](#how-to-use)
   - [Instalation](#installation)
   - [Test Code Explanation](#test-code-explanation)

## Motivation

The main goal of this project is to create an easy to build and 
assemble haptic installation which can be controlled as a haptic devise. 
This is made possible by the use of a joint with integrated springs. 
A greater displacement of the motor for a smaller amount of force 
compared to a rigid robotic arm is therefore possible. 

For how to use see: [How To Use](#how-to-use)

## Video
This YouTube video shows an example of both Impedance control and Admittance control was tested:
https://youtu.be/x0tvgowaUfE?si=nHkEfITcb-D-KBVI

## The Haptic Arm Lab

![Haptic Arm Lab](/images/DSC_2766.JPG)

In Haptic devices, there are two main ways for control. 
Impedance control aim to steer the position by reading the motor force. 
Admittance control aim to control the force of the device by adjusting the position. 
This two are integrated as methods and can be used directly, 
by the use of the low-level libraries this control codes may also be created by the user. 
A descriptive block diagram of the two control loops. 

![Block Diagram](/images/ControlStructureHapticArm.png)


## Code
The code which is to be used can be found in Code.

The main control .ino file can be altered to control the arm in different available modes. The libaries can also be used seperately for control of other mechanical instalation. 

For the Haptic Sommer School course use the studentProgram() implimented into the HapticArm libary for creating own self-produced controlling programs.

For more documentation of the system more detiled information can be found in the documentation folder. (LINK) The documentation is sectioned into mechanical buildt instruction, electrical components and softwear. In addition a more detaiueld explanation of the control theory is available.

The equations used for the control modes are shown beneath.

![Force Equations](/images/HapticForce.jpg)

## CAD
A version off the design can be found in CAD as .stl files ready for printing. In the detailed parts folder the smallest parts and the belt-gear 
which should be printed in higher detail for optimal use, can be found.

## Implementation of Seeeduino in Arduino IDE
To be able to access Seeduino XIAO in the Arduino IDe Board Manager, an "Additional Board Manager URLs:" have to be added in FIle > Preferences:
```
http://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json
```

## How To Use

### Installation

After the complete mechanical assembly is finished and the electronics
mounted as described [here](https://instructions.online/?id=3988-mechanical%20assembly),
the microcontroller of your choice can be plugged into your computer. An IDE for
programming microcontrollers are necessary, the Arduino IDE works perfectly fine and
can be downloaded [here](https://www.arduino.cc/en/software). 

After installing your IDE the seeeduino board programing package have to be added
as described in [Implementation of Seeeduino in Arduino IDE](#implementation-of-seeeduino-in-arduino-ide)
section. 

```text
http://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json
```

Go to Preferences and paste the Url given above into the "Additional board manager URLs:"
box. The seeeduino package should now be available to install from the board manager.

### Test Code

Start by cloning this repository on to your computer. Go to Code->testCode 
folder and open the testCode.ino file in the Arduino IDE. If the microcontroller
is connected, the port should be available in the drop-down menu at the top 
of the window. 

Choose which control method you want to applies by uncommenting the line providing
the selected method. For both ```goImpedance``` and ```goAdmittance``` functions
the three first arguments are respective mass, spring and damper coefficients
for your system. The last argument for impedance is optional and sets the initial
position. If no input are given, 125 degrees, which is the middle, is used. 
The last two arguments for Admittance is optional. The Initial position is 
default middle (125 degrees), and the initial force is 0 that represents the 
force sensitivity of the system. If the initial position is set to -1, the 
arm will hold the current position. This simulates a typical "tech the robot"
mode. 

The ```goPos``` function can be use for basic position control. This function
is used internally in the ```goAdmittance``` and should therefor be carefully 
tuned before moving on to the more advanced control methods. 

### Test Code Explanation

This code provides a basic setup for testing the pre-set control functions.
First, some basic parameters are set up and the hapticArm class used to 
crate an arm object. If any ports on the electronic setup are changed or the 
PID controller needs adjusting, these values can be altered to your preference.

```c++
#include "hapticarm.h"

#include "hapticsensor.h"

// Initiate ports numbers
int motorSetting[5] = {7, 8, 9, 2, 3};    // Pins for motor {forward, backward, PWM, hall_1, hall_2}
int sensorSetting[4] = {1, 0, 6, 10};      // Pins for sensor  {force, current, switch one, switch two}

// PID settings(Kp, Ki, Kd) {position, force, motor} 
float PIDset[3][3] = {{3,3,30},{3,1,0},{0,0,0}};

HapticArm HapArm_(motorSetting, sensorSetting, PIDset);

// Sinus wave
float A = -30;
float B = 0.1;
float dt = 0.0;
```

Every Arduino code contains two functions, the void setup and void loop. 
The setup function is used to setup parameters and input output settings. 
In the test code first a serial port is set up, enabling the serial communication
for feedback to the computer. Additionally, I2C communication is started with 
the ```Wire.begin()``` command. 

The calibration sequence is available in the HaticArm class. 
This actuates the arm to set the correct end switches and
set the encoder values for the end point. This is important since these values
are mapped to the corresponding arm angle which is used for position. 
The encoder limit values are printed in the serial monitor. If the values
are wraped around 0 the encoder chip should be rotated 90 degrees, as wrapping
of the encoder values has not yet been implemented. 

A loop is taken advantage of to set the Arm straight up which provides a 
smoother start for any actuation. This loop is stoped after approximately
10 seconds. Finally a Header to serial monitor with output explanation for the Admittance or
Impedance control function is available. This can be uncommented as needed. 

```c++
void setup() {
  // Initislize importent comunication
  Serial.begin(9600);
  Wire.begin();
  delay(2000);
  HapArm_.calibrateArm(); // If calibration is needed, else should be let out 
  delay(500);

  float curPos = 0;
  float startPos = 125;
  
  // Set arm to middle position
  while (curPos != startPos && dt < 10){
    HapArm_.goToPos(startPos);
    dt = dt + 0.001;
  }
  dt = 0.0;
  
  //Admittance header
  //Serial.println("AcutalPos;SetPos;SpringAngle;Force;RequestedPos;");

  //Impedace header
  Serial.println("AcutalPos;SetPos;MotorReqSpeed;TorqueMes;TorqueCalc;");
  
}
```

The void loop function will constantly run while the microcontroller is powered
on. Here the desired control function is implemented for the Arm object. 
The First line set a sine wave reference which could be used as a set point, for 
instance, seen in the [video](#video). 

The ```goPosition``` uses a basic PID 
control to set the desired position. The ```goImpedance``` function takes 
three required arguments mass, spring and damper coefficient of the system.
Additionally one optional argument can be given, namely the initial position. 
Default is 125 degrees (the middle), but any number between 0–250 can be used. 
The ```goAddmittance``` function takes three required arguments, mass, 
spring and damper coefficient of the system. In addition, two optional 
arguments can be set, initial position and initial force. The Initial position is
default in the middle but can be set to between 0–250. Initial force represents
the force sensitivity and is default 0.

```c++
void loop() {
  // sinewave for setopint reference
  float pos = A*sin(B*dt) + 125;

  // For control with Impedance uncomment this line
  //HapArm_.goImpedance(0, 0, 1);

  // For control with Admittance uncomment this line
  //HapArm_.goAdmittance(0.1,2,0.02,pos);

  // For basic position control uncomment this line
  HapArm_.goToPos(125);
}
```

 
