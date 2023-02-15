# Polar Robotics ESP32 Codebase
ONU Polar Robotics codebase for the ESP32 platform using the Arduino framework
contains the code needed to operate a 3-wheeled "tight-turn radius" robot to play in robotic football

## Espressif DevKitC Wroom-32s Pinout
<img src="/media/ESP32_DevkitC_Wroom32s_Pinout.jpg" 
  alt="DevkitC pinout"
  style="margin: 0 auto; width: 300px">

## NodeMCU-32s Pinout
<!--- 
https://m.media-amazon.com/images/I/516SPcBz+pL._AC_SY350_.jpg 
-->
<img src="/media/NodeMCU_32s_Pinout.jpg" 
  alt="NodeMCU pinout"
  style="margin: 0 auto; width: 300px">

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
├ Lights.cpp            Controls robot LED  (Removed... future location in PolarRobotics.h)
├ main.cpp              Contains code that initializes the Robot and Drivebase.
└ PolarRobotics.h       Contains globally relevant declarations and enums.
```
