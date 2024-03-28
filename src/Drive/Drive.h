#pragma once

#include <Arduino.h>
#include <Robot/MotorControl.h>
#include "PolarRobotics.h"

// Gyro Includes
#include <Adafruit_MPU6050.h>
#include <Wire.h>

#ifndef NUM_MOTORS
#define NUM_MOTORS 2
#endif // !NUM_MOTORS

// RAMP DEFINES
#ifndef ACCELERATION_RATE
// rate of change of power with respect to time when accelerating %power/10th of sec
#define ACCELERATION_RATE 0.00375f // [RPM/ms] possibly change to RPM/s for future
#endif // !ACCELERATION_RATE
// rate of deceleration/braking
#define BRAKE_PERCENTAGE 0.9
// how often the ramp() function changes the motor power
#define TIME_INCREMENT 5

// drive param generation
#define NORMAL_TURN_CONSTANT 0.05
// Value for the tank mode speed reduction percentage
#define TANK_MODE_PCT 0.75
// Value for the tank mode speed reduction percentage
#define RB_TANK_MODE_PCT 0.5
// Value for the Drift Mode Reduction Factor Percentage
#define DRIFT_MODE_PCT 0.8
//these should = normal speed, QB needs 0.5 for both 

// Controller Defines
#define STICK_DEADZONE 0.075 //0.0390625F // 8.0 / 127.0
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
#define SMALL_BOOST_PCT  0.85f
#define SMALL_NORMAL_PCT 0.7f
#define SMALL_SLOW_PCT   0.4f

// BSN for the 12v motors used on the new center
#define MECANUM_BOOST_PCT  0.8f
#define MECANUM_NORMAL_PCT 0.6f
#define MECANUM_SLOW_PCT   0.3f

// BSN for the falcon motors used on the runningback
#define FALCON_CALIBRATION_FACTOR 1.0f
#define FALCON_BOOST_PCT          0.6f
#define FALCON_NORMAL_PCT         0.4f // 0.5
#define FALCON_SLOW_PCT           0.15f

// BSN defines for the small 12v motors
#define SMALL_12V_BOOST_PCT          0.15f
#define SMALL_12V_NORMAL_PCT         0.1f // 0.5
#define SMALL_12V_SLOW_PCT           0.05f

#define BRAKE_BUTTON_PCT 0


class Drive {
  private:
    MotorType motorType; // TODO: Why is this private if we have a setter with no input validation? - MP 2023-05-10
    BotType botType; // TODO: I added this to private only because motorType was private.
    float gearRatio;
    bool hasEncoders;
    bool hasGyro;

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

    // Gyroscope
    Adafruit_MPU6050 mpu;
    sensors_event_t a, g, temp;
    float currentAngleSpeed;
    bool drivingStraight;
    int motorDiff;

    // PILoop
    bool CL_enable = true;
    int motorDiffCorrection;
    float k_p;
    float k_i;
    const float ERROR_THRESHOLD = 0.02;
    unsigned long lastTime;
    //integrate
    int prev_current_error;
    int integral_sum;
    unsigned long prev_integral_time;


    void calcTurning(float stickTrn, float fwdLinPwr);

  protected:
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
    Drive(BotType botType, MotorType motorType, drive_param_t driveParams, bool hasEncoders = false, int turnFunction = 2, bool hasGyro = false);
    void setupMotors(uint8_t lpin, uint8_t rpin);
    void setMotorType(MotorType motorType);
    void setStickPwr(int8_t leftY, int8_t rightX);
    float getForwardPower();
    float getTurnPower();
    void setBSN(Speed bsn); //(float powerMultiplier);
    void setBSNValue(float bsn_pct);
    float getBSN();
    void emergencyStop();
    void generateMotionValues(float tankModePct = TANK_MODE_PCT);
    virtual void update();
    void printSetup();
    virtual void printDebugInfo();
    virtual void printCsvInfo();

    // CL Gyro Functions
    void setCurrentAngleSpeed(float speed);
    int PILoop();
    int integrate(int current_error);
    void integrateReset();

    //* The following variables are initialized in the constructor
    // maximum speed for these is 1.0
    // percentage of power used when boosting for big motors
    float BIG_BOOST_PCT;
    
    // default: 0.6, this is the typical percentage of power out of the motors' range that is used (to ensure they don't do seven wheelies)
    float BIG_NORMAL_PCT;
    
    // should be a value less than BIG_NORMAL_PCT, to slow down for precision maneuvering, QB needs this to be 0.3
    float BIG_SLOW_PCT;
};