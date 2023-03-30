# Polar Robotics ESP32 Codebase
ONU Polar Robotics codebase for the ESP32 platform using the Arduino framework
contains the code needed to operate multiple postions of robots needed to play a robotic football game 

## Espressif DevKitC Wroom-32s Pinout
<img src="/media/ESP32-DevKitCPinout.png" 
  alt="DevkitC pinout"
  style="margin: 0 auto; width: 300px">

<!-- ## NodeMCU-32s Pinout

https://m.media-amazon.com/images/I/516SPcBz+pL._AC_SY350_.jpg 

<img src="/media/NodeMCU_32s_Pinout.jpg" 
  alt="NodeMCU pinout"
  style="margin: 0 auto; width: 600px"> -->

## Filestructure and Inheritance
```
src/
├ Drive/
│ ├ Drive.h              Header file for Drive.cpp
│ └ Drive.cpp            Standard drive code class/object
│   ├ DriveKicker.cpp    Special Drive for Kicker inherits from Drive
|   ├ DriveMecanum.h     Special Drive for New Center inherits from Drive
│   ├ DriveMecanum.cpp   Special Drive for New Center inherits from Drive
│   ├ DriveQuick.h       Special Drive for Runningback inherits from Drive
│   └ DriveQuick.cpp     Special Drive for Runningback inherits from Drive
│
├ Robot/
│ ├ Robot.cpp            Base Robot Parent Class
│ |  ├ Lineman.cpp       Lineman-specific functions, inherits from Robot
│ |  │ 
│ |  ├ Runningback.cpp   Runningback-specific functions, inherits from Robot
│ |  ├ Quarterback.cpp   Quarterback-specific functions and controls: flywheels, elevation, and conveyor
│ |  ├ Kicker.cpp        Kicker-specific functions and controls, incl. winding/release
│ |  ├ Center.cpp        Center-specific controls: claw and arm lift
│ |  └ CenterNew.cpp     New Center-specific functions, inherits from Robot
│ ├ Lights.cpp           Controls robot LEDs, indicating positon and status
| ├ MotorControl.h       Header for motor control class
| └ MotorControl.cpp     Definitions for motor control class, used by Drive & Robot classes
|
├ builtInLED.h           
├ builtInLED.cpp           
├ depairingStation.cpp   Contains main() code for a depairing station  
├ getAddress.cpp         Function to get the BT MAC of the ESP
├ pairing.h              Function prototypes for pairing
├ pairing.cpp            Function definitions used for pairing controllers to the esp
├ PolarRobotics.h        Contains globally relevant declarations and enums.
└ main.cpp               Contains code that initializes the Robot and Drivebase.
```
