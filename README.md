# Polar Robotics ESP32 Codebase
ONU Polar Robotics codebase based on the ESP32 framework 

## NodeMCU-32s Pinout
https://m.media-amazon.com/images/I/516SPcBz+pL._AC_SY350_.jpg 

## Filestructure and Inheritance
```
src/
├ Drive/
│ ├ Drive.h             Header file for Drive.cpp
│ └ Drive.cpp           Standard drive code class/object
│   ├ DriveKicker.cpp   Special Drive for Kicker
│   ├ DriveMecanum.cpp  Special Drive for New Center
│   └ DriveQuick.cpp    Special Drive for Runningback
│
├ Robot/
│ └ Robot.cpp           Base Robot Parent Class
│   ├ Lineman.cpp       Lineman-specific functions, inherits from Robot
│   │ └ Reciever.cpp    Receiver-specific functions, inherits from Lineman
│   ├ Runningback.cpp   Runningback-specific functions, inherits from Robot
│   ├ Quarterback.cpp   Quarterback-specific functions and controls: flywheels, elevation, and conveyor
│   ├ Kicker.cpp        Kicker-specific functions and controls, incl. winding/release
│   ├ Center.cpp        Center-specific controls: claw and arm lift
│   └ CenterNew.cpp     New Center-specific functions, inherits from Robot
│ 
├ Lights.cpp            Controls robot LEDs
├ main.cpp              Contains code that initializes the Robot and Drivebase.
└ PolarRobotics.h       Contains globally relevant declarations and enums.
```
