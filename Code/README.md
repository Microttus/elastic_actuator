# Haptic Arm Code

- The AD5600 libary are used for reading the Rotary Absolutt Encoder via I2C and are created by:
- The PID libary delivers different methods for PID calcuation and are created as objects
- The PWM motor libary are a compleate libary created for controlling a PWM motor in both directions (CW, CCW) and reading a attached incremental encoder
- The HaapticSensor libary are a range of methods for reading the spesific sensores attached to the Haptic Arm
- The HapticArm libary contains a range of methods for compleate control of the Haptic Arm using all the libaries metioned above

## HapticArm Libary

- The goToPos method makes use of a PID controller and the encoders for maintaing the position of the Arm at a sertain deegree from 0
- The resistForce makes the arm maintain the given position with the goToPos method until the force barrier given are breached, then the arm gived in and change position
- The goSpring method makes the Arm change its position form the given staring position acting like a spring with the given mass, dmaper and spring constant given
- The calicaration method makes use of the two presure switches for calibration of max and min calibration of the encoder
