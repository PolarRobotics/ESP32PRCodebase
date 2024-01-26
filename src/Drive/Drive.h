#pragma once

#include <Arduino.h>
#include <Robot/MotorControl.h>
#include "PolarRobotics.h"

#ifndef NUM_MOTORS
#define NUM_MOTORS 2
#endif // !NUM_MOTORS

// RAMP DEFINES
#ifndef ACCELERATION_RATE
// rate of change of power with respect to time when accelerating %power/10th of sec
// #define ACCELERATION_RATE .0375f // probably lower for runningback
#define ACCELERATION_RATE 0.00375f // probably lower for runningback
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

// Controller Defines
#define STICK_DEADZONE 0.035 //0.0390625F // 8.0 / 127.0
#define THRESHOLD 0.00001


// MOTOR MAX SPEED DEFINES;
// this is 1.0, the maximum power possible to the motors.
// #if BOT_TYPE == 4
// #define BIG_BOOST_PCT 0.7  // default: 0.6, this is the typical percentage of power out of the motors' range that is used (to ensure they don't do seven wheelies)
// #define BIG_NORMAL_PCT 0.3 // should be a value less than BIG_NORMAL_PCT, to slow down for precision maneuvering, QB needs this to be 0.3
// #define BIG_SLOW_PCT 0.2
// #else
// #define BIG_BOOST_PCT 0.7  // default: 0.6, this is the typical percentage of power out of the motors' range that is used (to ensure they don't do seven wheelies)
// #define BIG_NORMAL_PCT 0.6 // should be a value less than BIG_NORMAL_PCT, to slow down for precision maneuvering, QB needs this to be 0.3
// #define BIG_SLOW_PCT 0.3   // the value for brake button to slow down the motors at the button press
// #endif

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
#define FALCON_NORMAL_TREVOR_PCT 0.6 // used to make trevor(nt) happy
#define FALCON_NORMAL_PCT 0.4 // 0.5
#define FALCON_SLOW_PCT   0.15

#define BRAKE_BUTTON_PCT 0


class Drive {
  private:
    MotorType motorType; // TODO: Why is this private if we have a setter with no input validation? - MP 2023-05-10
    BotType botType; // TODO: I added this to private only because motorType was private.
    float gearRatio;
    bool hasEncoders;

    float BSNscalar;
    float wheelBase;
    int omega;
    int omega_L, omega_R;
    float R, R_Max, R_Min;
    int max_RPM, min_RPM;
    int enableTurnSensitivity;
    // Turn sensitivity variables
    float scaledSensitiveTurn = 0.0f;
    float turnSensitivityScalar = 0.0f;
    float domainAdjustment = 0.0f;

    unsigned long lastTime;
    float power;

    void calcTurning(float stickTrn, float fwdLinPwr);

  protected:
    // MotorControl* M1;
    // MotorControl* M2;
    MotorControl M1, M2;
    float stickForwardRev, stickTurn;
    float lastTurnPwr;
    float turnPower;

    float requestedMotorPower[NUM_MOTORS];
    float lastRampPower[NUM_MOTORS];
    float turnMotorValues[NUM_MOTORS];

  public:
    enum Speed {
        NORMAL,
        BOOST,
        SLOW,
        BRAKE
    };

    Drive();
    Drive(BotType botType, MotorType motorType);
    Drive(BotType botType, MotorType motorType, drive_param_t driveParams, bool hasEncoders = false);
    void setServos(uint8_t lpin, uint8_t rpin);
    void setServos(uint8_t lpin, uint8_t rpin, uint8_t left_enc_a_pin, uint8_t left_enc_b_pin, uint8_t right_enc_a_pin, uint8_t right_enc_b_pin);
    void setMotorType(MotorType motorType);
    void setStickPwr(int8_t leftY, int8_t rightX);
    float getForwardPower();
    float getTurnPower();
    void setBSN(Speed bsn); //(float powerMultiplier);
    void setBSNValue(float bsn_pct);
    float getBSN();
    void emergencyStop();
    void generateMotionValues();
    virtual void update(int speed);

    // here
    virtual void update2(int speedL, int speedR);

    void printSetup();
    virtual void printDebugInfo();

    //* The following variables are initialized in the constructor
    // maximum speed for these is 1.0
    // percentage of power used when boosting for big motors
    float BIG_BOOST_PCT;
    
    // default: 0.6, this is the typical percentage of power out of the motors' range that is used (to ensure they don't do seven wheelies)
    float BIG_NORMAL_PCT;
    
    // should be a value less than BIG_NORMAL_PCT, to slow down for precision maneuvering, QB needs this to be 0.3
    float BIG_SLOW_PCT;
};