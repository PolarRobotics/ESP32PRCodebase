#pragma once

#ifndef __OLDCENTER_H__
#define __OLDCENTER_H__


#include <Arduino.h>
#include <ESP32Servo.h>

enum armStatus {
  Higher, Lower, Stop, Hold
};

enum clawStatus {
  Open, Close, clawStop
};

class Center {
  private:
    // uint8_t clawPin, m_elevationpin;
    Servo clawmotor;
    Servo armmotor;

  public:
    Center(); 
    void setServos(Servo& armPin, Servo& clawPin);
    void clawControl(clawStatus reqstatus);
    void armControl(armStatus reqstatus);
};

#endif