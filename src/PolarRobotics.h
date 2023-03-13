#ifndef POLAR_ROBOTICS_H_
#define POLAR_ROBOTICS_H_

#include <Arduino.h>
// #include <Robot/Lights.h>

// Pin Declarations
#define LEFT_MOT_PIN  32  // Drive Base
#define RIGHT_MOT_PIN 33

#define SPECBOT_PIN1  0   // Special Bot Pins
#define SPECBOT_PIN2  0
#define SPECBOT_PIN3  0
#define SPECBOT_PIN4  0

#define CENT_ARM_PIN  0   // Old Center
#define CENT_CLAW_PIN 0

#define QB_FLYWH_PIN  0   // Quarterback
#define QB_CONV_PIN   0
#define QB_ELVMOT_PIN 0

#define KICK_WIND_PIN 0   // Kicker

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