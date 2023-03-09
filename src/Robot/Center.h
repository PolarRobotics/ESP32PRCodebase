#pragma once

#ifndef __OLDCENTER_H__
#define __OLDCENTER_H__

#include <Arduino.h>

enum armStatus {
  Higher, Lower, Stop, Hold
};

enum clawStatus {
  Open, Close, clawStop
};

class Center {
  private:
    // uint8_t clawPin, m_elevationpin;
    uint8_t motorPins[2];
    MotorControl clawMotor, armMotor;

  public:
    Center(); 
    void setServos(int armPin, int clawPin);
    void clawControl(clawStatus reqStatus);
    void armControl(armStatus reqStatus);
};

#endif