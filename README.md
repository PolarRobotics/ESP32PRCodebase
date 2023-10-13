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
  style="margin: 0 auto; width: 300px"> -->

## Filestructure and Inheritance
```
src
├── Drive
│   ├── Drive.cpp       
│   ├── Drive.h
│   ├── DriveMecanum.cpp
│   ├── DriveMecanum.h  
│   ├── DriveQuick.cpp
│   └── DriveQuick.h
├── Pairing
│   ├── depairingStation.cpp
│   ├── getAddress.cpp
│   ├── pairing.cpp
│   └── pairing.h
├── PolarRobotics.h
├── Robot
│   ├── Center.cpp
│   ├── Center.h
│   ├── Kicker.cpp
│   ├── Kicker.h
│   ├── Lights.cpp
│   ├── Lights.h
│   ├── Lineman.h
│   ├── MecanumCenter.cpp
│   ├── MecanumCenter.h
│   ├── MotorControl.cpp
│   ├── MotorControl.h
│   ├── Quarterback.cpp
│   ├── Quarterback.h
│   ├── Robot.h
│   ├── builtInLED.cpp
│   └── builtInLED.h
├── Utilities
│   ├── BotTypes.cpp
│   ├── BotTypes.h
│   ├── ConfigManager.cpp
│   ├── ConfigManager.h
│   ├── MotorTypes.cpp
│   ├── MotorTypes.h
│   ├── Pair.h
│   ├── ReadBotInfo.cpp
│   └── WriteBotInfo.cpp
└── main.cpp
```
