#pragma once

#ifndef KICKER_H
#define KICKER_H

#include <Arduino.h>
#include <Robot/MotorControl.h>
#include <ps5Controller.h> // ESP PS5 library, access using global instance `ps5`

/**
 * @brief Kicker header file
 * @authors Andrew Nelson 
 */

// TODO: rename all variables to remove any `m_` prefixes and use proper camelCase
// TODO: move definitions from class and this header file to .cpp file

class Kicker : public Robot {
  private:
    bool enabled; // safety feature
    uint8_t kickerPin;
    MotorControl windupMotor;
  public:
    Kicker(uint8_t kicker_pin);
    void action() override; //! robot subclass must override action
    void enable();
    void test(); 
    void turnfwd(); 
    void turnrev(); 
    void stop(); 
};

#endif // KICKER_H
