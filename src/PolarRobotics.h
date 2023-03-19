#ifndef POLAR_ROBOTICS_H_
#define POLAR_ROBOTICS_H_

#include <Arduino.h>
// #include <Robot/Lights.h>

// Pin Declarations  |     configurations     |
//     | mot# |##|   | 2 wheels  |  4 wheels  |
// ____|______|__|___|___________|____________|
#define M1_PIN 32 // | leftDriv  | leftFront  |
#define M2_PIN 33 // | rightDriv | rightFront |
#define M3_PIN 26 // |    N/A    | leftRear   |
#define M4_PIN 27 // |    N/A    | rightRear  |

#define SPECBOT_PIN1  18   // Special Bot Pins
#define SPECBOT_PIN2  19
#define SPECBOT_PIN3  21
#define SPECBOT_PIN4  22

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
  big, // MOTOR_TYPE value of 0 for long/big motors
  small, // MOTOR_TYPE value of 1 for short/small motors
  mecanummotor, // MOTOR_TYPE value of 2 for small 12v mecanum motors
  falconmotor // MOTOR_TYPE value of 3 for the falcon motors on the runningback
};

#endif