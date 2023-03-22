#pragma once

#ifndef DRIVE_H_
#define DRIVE_H_

#include <Arduino.h>
#include "PolarRobotics.h"
#include "Robot/MotorControl.h"

#ifndef NUM_MOTORS
// the number of motors associated with driving
#define NUM_MOTORS 2
#endif

// rate of change of power with respect to time when accelerating %power/10th of sec
#define ACCELERATION_RATE .0375
// rate of deceleration/braking
#define BRAKE_PERCENTAGE 0.9
// how often the ramp() function changes the motor power
#define TIME_INCREMENT 25
#define BREAK_TIME_INCREMENT 12.5
// DO NOT CHANGE THIS EVER!!!!!
#define PWM_CONVERSION_FACTOR 0.3543307087

#define NORMAL_TURN_CONSTANT 0.05

// Controller Defines
#define turnMax 0.3 // the max allowable turning when the bot is traveling at lowest speed
#define turnMin 0.1 // the min allowable turning when the bot is traveling at full speed
#define STICK_DEADZONE 0.0390625F // 8.0 / 127.0
#define THRESHOLD 0.00001

// this is 1.0, the maximum power possible to the motors.
#define BIG_BOOST_PCT 0.7  // default: 0.6, this is the typical percentage of power out of the motors' range that is used (to ensure they don't do seven wheelies)
#define BIG_NORMAL_PCT 0.4 // should be a value less than BIG_NORMAL_PCT, to slow down for precision maneuvering
#define BIG_SLOW_PCT 0.2   // the value for brake button to slow down the motors at the button press
#define BRAKE_BUTTON_PCT 0

// BSN for Short/Small Motors
#define SMALL_BOOST_PCT 0.7
#define SMALL_NORMAL_PCT 0.5
#define SMALL_SLOW_PCT 0.2

// BSN for the 12v motors used on the new center
#define MECANUM_BOOST_PCT  0.8
#define MECANUM_NORMAL_PCT 0.6
#define MECANUM_SLOW_PCT   0.3

// BSN for the falcon motors used on the runningback
#define FALCON_BOOST_PCT  0.95
#define FALCON_NORMAL_PCT 0.8
#define FALCON_SLOW_PCT   0.4

#define BRAKE_BUTTON_PCT 0

class Drive {
private:
    MOTORS motorType;
    uint8_t motorPins[NUM_MOTORS];
    MotorControl M1, M2;

    float stickForwardRev, stickTurn;
    float BSNscalar;
    float lastTurnPwr;
    float turnPower;
    
    unsigned long lastRampTime[NUM_MOTORS];
    float motorPower[NUM_MOTORS];
    float currentPower[NUM_MOTORS];
    float lastRampPower[NUM_MOTORS];
    float turnMotorValues[NUM_MOTORS];
    // float inputPower[NUM_MOTORS];
    // float rampedPower[NUM_MOTORS];
    void calcTurningMotorValues(float stickTrn,  float prevPwr, int dir);
    void generateMotionValues();
    float ramp(float requestedPower, uint8_t mtr);
public:
    enum SPEED {
        normal,
        boost,
        slow,
        brake
    };
    Drive();
    void setServos(uint8_t lpin, uint8_t rpin);
    void setMotorType(MOTORS motorType);
    void attach();
    void setStickPwr(int8_t leftY, int8_t rightX);
    void setBSN(SPEED bsn); //(float powerMultiplier);
    float getMotorPwr(uint8_t mtr);
    void emergencyStop();
    void update();
    void drift();
    void printDebugInfo();
};

#endif /* DRIVE_H */
