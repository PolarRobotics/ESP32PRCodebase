#pragma once

#ifndef DRIVE_H_
#define DRIVE_H_

#include <Arduino.h>
#include "PolarRobotics.h"
#include "Robot/MotorControl.h"


#ifndef NUM_MOTORS
#define NUM_MOTORS 2
#endif // !NUM_MOTORS

// RAMP DEFINES
#ifndef ACCELERATION_RATE
// rate of change of power with respect to time when accelerating
#define ACCELERATION_RATE 0.0375f // probably lower for runningback
#endif // !ACCELERATION_RATE
// rate of deceleration/braking
#define BRAKE_PERCENTAGE 0.9
// how often the ramp() function changes the motor power
#define TIME_INCREMENT 5


// drive param generation
#define NORMAL_TURN_CONSTANT 0.05
// Value for the tank mode speed reduction percentage
#define TANK_MODE_PCT 0.75
// Value for the Drift Mode Reduction Factor Percentage
#define DRIFT_MODE_PCT 0.8
//these should = normal speed, QB needs 0.5 for both 


#if BOT_TYPE == 1
#define turnMax 0.8 
#define turnMin 0.8 
#elif BOT_TYPE == 2
#define turnMax 0.2 
#define turnMin 0.2 
#elif BOT_TYPE == 4
#define turnMax 0.4
#define turnMin 0.4
#elif BOT_TYPE == 6
#define turnMax 0.5
#define turnMin 0.2
#else
#define turnMax 0.65 // the max allowable turning when the bot is traveling at lowest speed
#define turnMin 0.65 // the min allowable turning when the bot is traveling at full speed
#endif

// Controller Defines
#define STICK_DEADZONE 0.0390625F // 8.0 / 127.0
#define THRESHOLD 0.00001


// MOTOR MAX SPEED DEFINES;
// this is 1.0, the maximum power possible to the motors.
#if BOT_TYPE == 4
#define BIG_BOOST_PCT 0.7  // default: 0.6, this is the typical percentage of power out of the motors' range that is used (to ensure they don't do seven wheelies)
#define BIG_NORMAL_PCT 0.3 // should be a value less than BIG_NORMAL_PCT, to slow down for precision maneuvering, QB needs this to be 0.3
#define BIG_SLOW_PCT 0.2
#else
#define BIG_BOOST_PCT 0.7  // default: 0.6, this is the typical percentage of power out of the motors' range that is used (to ensure they don't do seven wheelies)
#define BIG_NORMAL_PCT 0.6 // should be a value less than BIG_NORMAL_PCT, to slow down for precision maneuvering, QB needs this to be 0.3
#define BIG_SLOW_PCT 0.3   // the value for brake button to slow down the motors at the button press
#endif

// BSN for Short/Small Motors
#define SMALL_BOOST_PCT 0.85
#define SMALL_NORMAL_PCT 0.7
#define SMALL_SLOW_PCT 0.4

// BSN for the 12v motors used on the new center
#define MECANUM_BOOST_PCT  0.8
#define MECANUM_NORMAL_PCT 0.6
#define MECANUM_SLOW_PCT   0.3

// BSN for the falcon motors used on the runningback
#define FALCON_BOOST_PCT  1.0
#define FALCON_NORMAL_PCT 0.4
#define FALCON_SLOW_PCT   0.3

#define BRAKE_BUTTON_PCT 0

class Drive {
protected:
    MotorControl M1, M2;
    float stickForwardRev, stickTurn;
    float lastTurnPwr;
    float turnPower;
    unsigned long lastRampTime[NUM_MOTORS];
private:
    eMOTOR_TYPE motorType;
    float BSNscalar;
    float requestedMotorPower[NUM_MOTORS];
    float currentRampPower[NUM_MOTORS];
    float lastRampPower[NUM_MOTORS];
    float turnMotorValues[NUM_MOTORS];
    void calcTurningMotorValues(float stickTrn, float prevPwr, int dir);
public:

    enum SPEED {
        normal,
        boost,
        slow,
        brake
    };
    Drive();
    void setServos(uint8_t lpin, uint8_t rpin);
    void setMotorType(eMOTOR_TYPE motorType);
    void setStickPwr(int8_t leftY, int8_t rightX);
    float getForwardPower();
    float getTurnPower();
    void setBSN(SPEED bsn); //(float powerMultiplier);
    float getBSN();
    float getReqMotorPwr(uint8_t mtr);
    void setReqMotorPwr(float power, uint8_t mtr);
    void setLastRampPwr(float power, uint8_t mtr);
    void emergencyStop();
    float ramp(float requestedPower, uint8_t mtr, float accelRate = ACCELERATION_RATE);
    void generateMotionValues();
    void update();
    void printDebugInfo();
};

#endif /* DRIVE_H */
