#pragma once

#ifndef QUARTERBACK_BASE_H
#define QUARTERBACK_BASE_H

#include <Robot/Robot.h>
#include <Robot/MotorControl.h>
#include <Drive/Drive.h>

// Wifi Connectivity Pin
#define WIFI_PIN 18

//UART Pins
#define RX2 16
#define TX2 17

#include <HardwareSerial.h>


/**
 * @brief Quarterback Base Subclass Header
 * @authors Max Phillips, George Rak, Corbin Hibler
 */
class QuarterbackBase : public Robot {
  private: 

    Drive* drive;

    /* VARIABLES FOR WIFI CONNECTIVITY
        - targetValue:                The current drive motor value
        - UARTMessage:                The string version of the message to send from the base of the QB to it's server ESP also mounted on the base
    */
    int targetValue = 0;
    String UARTMessage = "";

  public:
    QuarterbackBase (Drive* drive);
    void action() override; //! robot subclass must override action
    void updateWriteMotorValues();
    void bottomQBSetup();
};

#endif // QUARTERBACK_H
