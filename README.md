# HapticSommerSchool

The main gloa of this project is to create a easy to bulid and assamble haptic instalation which can be controlled as a haptic devise. This is made possible by the use of a joit with integrated springs. A greater displacment of the motor for a smaller amount of force compeared to a rigid robotic arm are therfor possible. 

## The Haptic Arm Lab

![Haptic Arm Lab](/images/DSC_2766.JPG)

In Haptic devices there are two main ways for control. Impedance control aim to steer the position by reading the motor force. Admittance control aim to control the force of the device by adjusting the position. This two are integrated as methods and can be used directly, By the use of the low level libaries this contol codes may also be created by the user. A discriptive block diagram of the two control loops. 

![Block Diagram](/images/AdImpblockdiagram.jpg)


## Code
The code which is to be used can be found in Code.

The main control .ino file can be altered to control the arm in different available modes. The libaries can also be used seperately for control of other mechanical instalation. 

For the Haptic Sommer School course use the studentProgram() implimented into the HapticArm libary for creating own self-produced controlling programs.

For more documentation of the system more detiled information can be found in the documentation folder. (LINK) The documentation is sectioned into mechanical buildt instruction, electrical components and softwear. In addition a more detaiueld explanation of the control theory is available.

The equations used for the control modes are shown beneath.

![Force Equations](/images/HapticForce.jpg)

## CAD
A version off the design can be found in CAD as .stl files ready for printing. In the detailed parts folder the smallest parts and the belt-gear which should be printed in higher detail for optimale use, can be found.

## Implimantation od Seeeduino in Arduino IDE
To be able to access Seeduino XIAO in the Arduino IDe Board Manager, an "Additional Board Manager URLs:" have to be added in FIle > Preferences:
```
http://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json
```
