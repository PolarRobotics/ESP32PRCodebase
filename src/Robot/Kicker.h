#pragma once

#ifndef KICKER_H_
#define KICKER_H_

#include <Arduino.h>

/**
 * @brief Kicker header file
 * @authors Andrew Nelson 
 */

class Kicker { //: public Robot
  private:
    bool m_enabled; // safety feature
    uint8_t m_kickerpin;
    Servo m_windupMotor;
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
        m_windupMotor.write(50); //clockwise
        delay(3000);
        m_windupMotor.write(90); //stop
        delay(1000);
        m_windupMotor.write(130); //counter-clockwise
        delay(3000);
        m_windupMotor.write(90); //stop
      }
    }
    void turnfwd() {
      if (m_enabled)
        m_windupMotor.write(70);
    }
    void turnrev() {
      if (m_enabled)
        m_windupMotor.write(110);
    }

    void stop() {
      if (m_enabled)
        m_windupMotor.write(90);
    }
};

#endif /* KICKER_H_ */