#pragma once

#ifndef DRIVE_MECANUM_H_
#define DRIVE_MECANUM_H_

#include <Arduino.h>

//! this doesn't work with new OOP. -MP 2023-05-01
// keep these defines before drive is included to override default vaules
#define MC_NUM_MOTORS 4
#define MC_ACCELERATION_RATE .0375

// #define MECANUM_MOT_BOOST_PCT  0.8
// #define MECANUM_MOT_NORMAL_PCT 0.6
// #define MECANUM_MOT_SLOW_PCT   0.4

#include <Drive/Drive.h>



class DriveMecanum : public Drive {
private:
    MotorControl LF, RF, LR, RR;
    float mmotorpwr[MC_NUM_MOTORS];
    float scaledLeftX;
    float scaledLeftY;
    float scaledRightX;
    float r; // r the magnitude of travel
    float theta; // the angle of travel
    float turnPwr; // used to turn the bot
    float x_comp, y_comp, max;
    //unsigned long lastRampTime; //only needs to be one value
    // float motorPwr[MC_NUM_MOTORS]; // declared in parent class, use accessors
    void generateMotorValues();
public:
    DriveMecanum();
    void setServos(uint8_t lfpin, uint8_t rfpin, uint8_t lrpin, uint8_t rrpin);
    void setStickPwr(int8_t leftX, int8_t leftY, int8_t rightX);
    // float getReqMotorPwr(uint8_t mtr);
    void emergencyStop();
    void update();
    void drift();
    void printDebugInfo();
};

#endif