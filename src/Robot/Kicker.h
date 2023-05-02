#pragma once

#ifndef KICKER_H_
#define KICKER_H_

#include <Arduino.h>
#include <Robot/MotorControl.h>

/**
 * @brief Kicker header file
 * @authors Andrew Nelson 
 */

// TODO: rename all variables to remove any `m_` prefixes and use proper camelCase
// TODO: move definitions from class and this header file to .cpp file

class Kicker : public Robot {
  private:
    bool m_enabled; // safety feature
    uint8_t m_kickerpin;
    MotorControl m_windupMotor;
  public:
    Kicker() {
      m_enabled = false;
    }
    void action() override; //! robot subclass must override action
    void setup(uint8_t kicker_pin) {
      m_enabled = true;
      // TODO: move kicker pin arg, assignment, and attachment to constructor
      // TODO: rename setup() to enable(), have it only change `enabled`, and call it in main *after* pairing completes if the bot type is kicker
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

void Kicker::action() {
  // Control the motor on the kicker
  if (ps5.Triangle())
      turnfwd();
  else if (ps5.Cross())
      turnrev();
  else
      stop();
}

#endif /* KICKER_H_ */