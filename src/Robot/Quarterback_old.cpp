#include "PolarRobotics.h"
#include <Arduino.h>
// #include <Servo.h>

// TODO: Can we get rid of this file?

/*
       ___    _   _      _      ____    _____   _____   ____    ____       _       ____   _  __   ____         ___
      / _ \  | | | |    / \    |  _ \  |_   _| | ____| |  _ \  | __ )     / \     / ___| | |/ /  |___ \       / _ \
     | | | | | | | |   / _ \   | |_) |   | |   |  _|   | |_) | |  _ \    / _ \   | |     | ' /     __) |     | | | |
     | |_| | | |_| |  / ___ \  |  _ <    | |   | |___  |  _ <  | |_) |  / ___ \  | |___  | . \    / __/   _  | |_| |
      \___\_\ \___/  /_/   \_\ |_| \_\   |_|   |_____| |_| \_\ |____/  /_/   \_\  \____| |_|\_\  |_____| (_)  \___/
  */


// QB Aim Enum
// Elevation enum and variables for linear actuators
enum ELEVATION {
  low,
  middle,
  high
};

// Pin and Servo for quarterback flywheels
int rightFlywheelPin = 18;
int leftFlywheelPin = 19;
// Servo rightFlywheelMotor;
// Servo leftFlywheelMotor;

// Pin and Servo for quarterback firing mechanism
int FirePin = 16;
// Servo FireMotor;

// Pin and Servo for quarterback aiming mechanism
int ElevationPin = 14;
// Servo ElevationMotor;

// Servo flywheels;
// Servo conveyor;

// Servo getElevationMotor() { return ElevationMotor; }

// this function contains everything related to QB from main.cpp
void setup() {
  // flywheels.attach(4);
  // conveyor.attach(2);
  // rightFlywheelMotor.attach(rightFlywheelPin);
  // leftFlywheelMotor.attach(leftFlywheelPin);
  // FireMotor.attach(FirePin);
  // ElevationMotor.attach(ElevationPin);
  // conveyor.write(30);
}

// /* * * * * * * * * * * * * * *
//  * Quarterback Internal Code *
//  * * * * * * * * * * * * * * */

// Servo Speed Consts
const int SERVO_SPEED_UP = 175; // this should be between 90 and 180.
const int SERVO_SPEED_STOP = 90; // this should always be 90.
const int SERVO_SPEED_DOWN = 5; // this should be between 0 and 90.
int getSpeedUp() { return SERVO_SPEED_UP; }
int getSpeedStop() { return SERVO_SPEED_STOP; }
int getSpeedDown() { return SERVO_SPEED_DOWN; }

// Flywheel Speed Consts
const int FLYWHEEL_RIGHT_SPEED_FULL = 20; // this should be between 0 and 90.
const int FLYWHEEL_LEFT_SPEED_FULL = 170; // this should be between 90 and 180.
const int FLYWHEEL_STOP_SPEED = 90; // this should always be 90.
// to set the flywheel to another speed, subtract (for right) or add (left) from/to FLYWHEEL_STOP_SPEED like so:
// for example, if you wanted 10 power (about 14% of the range between off and full)
// leftFlywheelMotor.write(FLYWHEEL_STOP_SPEED + 10);

// Elevation Int
const int QB_ELEVATION_INTERVAL = 4000; // Constant for time to determine stages of elevation
unsigned long getElevationInterval() { return QB_ELEVATION_INTERVAL; }

// Elevation Time Benchmark (interval * stage)
unsigned long benchmark = 0;
unsigned long getAimBenchmark() { return benchmark; }

/* * * * * * * * * * * * * * *
 * Aiming / Elevation Control *
 * * * * * * * * * * * * * * */

// stage is the difference between the current position and target position, from a value of [-2, 2]
void aim(int stage) {
  benchmark = abs(stage) * QB_ELEVATION_INTERVAL;
  if (stage > 0) { // target position above current position
    // getElevationMotor().write(SERVO_SPEED_UP);
  } else if (stage < 0) { // target position below current position
    // getElevationMotor().write(SERVO_SPEED_DOWN);
  }
}

void stopAiming() { // wonder what this does
  // turn elevation motor off
  // getElevationMotor().write(SERVO_SPEED_STOP);
}

//debug
void debug_showSelectionInfo(ELEVATION c, ELEVATION t) {
  Serial.print("\n");
  Serial.print("current: ");
  Serial.println(c);
  Serial.print("target: ");
  Serial.println(t);
  Serial.print("abs: ");
  Serial.println(abs(c - t));
}

void debug_showBenchmark() {
  Serial.print("\n");
  Serial.print("benchmark: ");
  Serial.println(benchmark);
  Serial.print("\n");
}

void debug_showTime() {
  Serial.print("\n");
  Serial.print("millis(): ");
  Serial.println(millis());
  Serial.print("\n");
  Serial.print("timestamp: ");
  // Serial.println(QBAimTimestamp);
}

/* * * * * * * * * *
 * Flywheel Control *
 * * * * * * * * * */

// /** startFlywheel */
// void startFlywheel(Servo rightFlywheelMotor, Servo leftFlywheelMotor) {
//   rightFlywheelMotor.write(FLYWHEEL_RIGHT_SPEED_FULL);
//   leftFlywheelMotor.write(FLYWHEEL_LEFT_SPEED_FULL);
// }

// /** stopFlywheel */
// void stopFlywheel(Servo rightFlywheelMotor, Servo leftFlywheelMotor) {
//   rightFlywheelMotor.write(FLYWHEEL_STOP_SPEED);
//   leftFlywheelMotor.write(FLYWHEEL_STOP_SPEED);
// }

// /** passBall */
// // pass the ball to the kicker
// void passBall(Servo rightFlywheelMotor, Servo leftFlywheelMotor) {
//   //turn flywheels on to low: approx 10 power for a light boost
//   rightFlywheelMotor.write(FLYWHEEL_STOP_SPEED - 10);
//   leftFlywheelMotor.write(FLYWHEEL_STOP_SPEED + 10);
//   //once firing mechanism is finished add that in and make it a macro?
// }

/* * * * * * * * * * * * * *
 * Firing Mechanism Control *
 * * * * * * * * * * * * * */

/** fireWeapon */
// void fireWeapon(Servo FireMotor, String requestedStatus) {
//   if (requestedStatus == "Fire") {
//     FireMotor.write(50);
//   } else if (requestedStatus == "Retract") {
//     FireMotor.write(130);
//   } else if (requestedStatus == "Stop") {
//     FireMotor.write(90);
//   }
// }