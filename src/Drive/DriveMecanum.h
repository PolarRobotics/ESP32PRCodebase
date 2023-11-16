#pragma once

#ifndef DRIVE_MECANUM_H
#define DRIVE_MECANUM_H

#include <Arduino.h>
#include <Drive/Drive.h>

#define MC_NUM_MOTORS 4
#define MC_ACCELERATION_RATE .0375

//! for mecanum BSN values, see Drive.h

class DriveMecanum : public Drive {
  private:
    MotorControl LF, RF, // front motors 
                 LB, RB; // back motors
    float mecanumMotorPwr[MC_NUM_MOTORS];
    float stickStrafe;
    float stickForward;
    float stickTurn;
    float r; // r the magnitude of travel
    float theta; // the angle of travel
    float turnPwr; // used to turn the bot
    float x_comp, y_comp, max;
    // unsigned long lastRampTime; // only needs to be one value
    // float motorPwr[MC_NUM_MOTORS]; // declared in parent class, use accessors
    void generateMotorValues();
    void generateMotorValuesOld();
  public:
    //! Must call base class constructor with appropriate arguments
    DriveMecanum(); // This is the prototype, see DriveMecanum.cpp for implementation
    void setServos(uint8_t pinLF, uint8_t pinRF, uint8_t pinLB, uint8_t pinRB);
    void setStickPwr(int8_t leftX, int8_t leftY, int8_t rightX);
    // float getReqMotorPwr(uint8_t mtr);
    void emergencyStop();
    void update();
    void printDebugInfo();
};

#endif // DRIVE_MECANUM_H