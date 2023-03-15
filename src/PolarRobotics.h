#ifndef POLAR_ROBOTICS_H_
#define POLAR_ROBOTICS_H_

#include <Arduino.h>
// #include <Robot/Lights.h>

// Pin Declarations
#define LEFT_MOT_PIN  32  // Drive Base
#define RIGHT_MOT_PIN 33

#define SPECBOT_PIN1  18   // Special Bot Pins
#define SPECBOT_PIN2  19
#define SPECBOT_PIN3  4
#define SPECBOT_PIN4  2

// Robot Type Enum
// 0 for lineman, 1 for reciever, 2 for center, 3 for quarterback, 4 for kicker
enum TYPE {
  lineman,
  receiver,
  center,
  quarterback,
  kicker
};

enum BOT_STATE {
  PAIRING,
  CONNECTED,
  DISCONNECTED,
  OFFENSE,
  DEFENSE,
  TACKLED
};

enum AGE {
  OLD,
  NEW
};

enum ELEVATION {
  low,
  high
};

enum MOTORS {
  big, // EEPROM value of 0 for long/big motors
  small // EEPROM value of 1 for short/small motors
};

#endif