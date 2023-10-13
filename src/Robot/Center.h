#pragma once

#ifndef OLD_CENTER_H
#define OLD_CENTER_H

#include <Robot/Robot.h>
#include <Robot/MotorControl.h>
#include <ps5Controller.h> // ESP PS5 library, access using global instance `ps5`

enum ArmStatus {
  HIGHER, LOWER, STOP_ARM, HOLD
};

enum ClawStatus {
  OPEN, CLOSE, STOP_CLAW
};

class Center : public Robot {
  private:
    uint8_t armPin, clawPin;
    MotorControl clawMotor, armMotor;

  public:
    Center(uint8_t armPin, uint8_t clawPin); 
    void action() override; //! robot subclass must override action
    void clawControl(ClawStatus reqStatus);
    void armControl(ArmStatus reqStatus);
};

#endif // OLD_CENTER_H