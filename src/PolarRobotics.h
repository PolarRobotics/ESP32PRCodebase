/**
 * @brief Polar Robotics Main Header File
 * 
 * Contains pin declarations/defines, various global enums, 
 * and other structures that are useful in a global scope.
 **/

#ifndef POLAR_ROBOTICS_H_
#define POLAR_ROBOTICS_H_

#include <Arduino.h>
#include <ps5Controller.h>

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
#define SPECBOT_PIN1 18   // Special Bot Pins
#define SPECBOT_PIN2 19
#define SPECBOT_PIN3 21
#define SPECBOT_PIN4 22

// pin for ws2812 LEDs to indicate positions 
#define LED_PIN 4   
// receiver, tackled, etc...
#if BOT_TYPE == 6
#define TACKLE_PIN 12
#else
#define TACKLE_PIN 13
#endif

// pairing jumper pin
// jump HIGH to activate pairing discovery
// otherwise robot will only connect to last controller
#define PAIRING_PIN 23


// TODO: Some of our enums are capitalized, some are not. We should probably standardize this.

// Robot Type Enum
// Ordinals: 0 for lineman, 1 for reciever, 2 for center, 3 for quarterback, 4 for kicker
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
  big, // MOTOR_TYPE value of 0 for long/big motors
  small, // MOTOR_TYPE value of 1 for short/small motors
  mecanummotor, // MOTOR_TYPE value of 2 for small 12v mecanum motors
  falconmotor // MOTOR_TYPE value of 3 for the falcon motors on the runningback
};

// external function implemented in main to allow accessing LED state from other files,
// like `pairing.cpp`, that would otherwise cause circular dependencies
extern void extUpdateLEDs(); 

#endif