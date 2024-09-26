/**
 * @brief Polar Robotics Main Header File
 * 
 * Contains pin declarations/defines, various global enums, 
 * and other structures that are useful in a global scope.
 **/

#ifndef POLAR_ROBOTICS_H
#define POLAR_ROBOTICS_H

#include <Arduino.h>
#include <Utilities/BotTypes.h>
#include <Utilities/MotorTypes.h>
#include <Utilities/DriveParameters.h>

// #define PR_CODEBASE_VERSION "2.4.5 @ production"

// PR_CODEBASE_VERSION is now defined dynamically at the start of the build process
#ifndef PR_CODEBASE_VERSION
#define PR_CODEBASE_VERSION "PR_CODEBASE_VERSION not defined!"
#endif

// [PIN DECLARATIONS]
// please follow: 
// https://docs.google.com/spreadsheets/d/17pdff4T_3GTAkoctwm2IMg07Znoo-iJkyDGN5CqXq3w/edit#gid=0
//
//  Motor and Pin #  |     Configurations     |
//     | mot# |##|   | 2 wheels  |  4 wheels  |
// ____|______|__|___|___________|____________|
#define M1_PIN 32 // | leftDriv  | leftFront  |
#define M2_PIN 33 // | rightDriv | rightFront |
#define M3_PIN 26 // |    N/A    | leftRear   |
#define M4_PIN 27 // |    N/A    | rightRear  |

// Pins for special bot features, conveyor, flywheels, etc...
#define SPECBOT_PIN1 18
#define SPECBOT_PIN2 19
#define SPECBOT_PIN3 21
#define SPECBOT_PIN4 22

// Encoder pin definitions
#define ENC1_CHA 35
#define ENC1_CHB 34
#define ENC2_CHA VP
#define ENC2_CHB VN

// pin for ws2812 LEDs to indicate positions 
#define LED_PIN 4   
// receiver, tackled, etc...
#define TACKLE_PIN 13

enum BOT_STATE {
  PAIRING,
  CONNECTED,
  DISCONNECTED,
  OFFENSE,
  DEFENSE,
  TACKLED
};

#endif // POLAR_ROBOTICS_H