#pragma once

#ifndef KICKER_H
#define KICKER_H

#include <Arduino.h>
#include <Robot/MotorControl.h>

/**
 * @brief Kicker header file
 * @authors Andrew Nelson 
 */

class Kicker { //: public Robot
  private:
    bool m_enabled; // safety feature
    uint8_t m_kickerpin;
    MotorControl m_windupMotor;
  public:
    Kicker() {
      m_enabled = false;
    }
    void setup(uint8_t kicker_pin) {
      m_enabled = true;
      m_kickerpin = kicker_pin;
      m_windupMotor.attach(kicker_pin);
    }
    void Test() {
      if (m_enabled) {
        m_windupMotor.write(-1); //clockwise
        delay(3000);
        m_windupMotor.write(0); //stop
        delay(1000);
        m_windupMotor.write(1); //counter-clockwise
        delay(3000);
        m_windupMotor.write(0); //stop
      }
    }
    void turnfwd() {
      if (m_enabled)
        m_windupMotor.write(-0.5);
    }
    void turnrev() {
      if (m_enabled)
        m_windupMotor.write(0.5);
    }

    void stop() {
      if (m_enabled)
        m_windupMotor.write(0);
    }
};

#endif // KICKER_H