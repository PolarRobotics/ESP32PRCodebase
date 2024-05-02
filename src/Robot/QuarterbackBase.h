#pragma once

#ifndef QUARTERBACK_BASE_H
#define QUARTERBACK_BASE_H

#include <Robot/Robot.h>
#include <Robot/MotorControl.h>
#include <Drive/Drive.h>

// Wifi Connectivity Pin
#define WIFI_PIN 18


/**
 * @brief Quarterback Base Subclass Header
 * @authors Max Phillips, George Rak, Corbin Hibler
 */
class QuarterbackBase : public Robot {
  private: 

    Drive* drive;

    /* VARIABLES FOR WIFI CONNECTIVITY
        - currentUpdateMotorMillis:   used to time when functions activate
        - targetValue:                The current value being sent by the timed function so testAnalogOutput can be updated while target value does not fluctuate mid send
        - prevUpdateTargetMillis:     Used for timing of functions
        - timesSentSession:           tracking how many times we have sent a value out of the targetValue
    */
    unsigned long currentUpdateMotorMillis = millis();
    int targetValue = 0;
    unsigned long prevUpdateTargetMillis = 0;
    int timesSentSession = 0;
    unsigned long previousMillis = 0;

  public:
    QuarterbackBase (Drive* drive);
    void action() override; //! robot subclass must override action
    void updateWriteMotorValues();
    int checkGetNewTarget();
    void bottomQBSetup();
};

#endif // QUARTERBACK_H
