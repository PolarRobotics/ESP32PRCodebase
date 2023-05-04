#pragma once

#ifndef OLD_CENTER_H
#define OLD_CENTER_H

#include <Arduino.h>
#include "Robot/MotorControl.h"

// TODO: Move definitions to .cpp
// TODO: Standardize commenting using doxygen (see builtInLED.h, pairing.cpp)

// TODO: stylize enums properly
enum armStatus {
  Higher, Lower, Stop, Hold
};

enum clawStatus {
  Open, Close, clawStop
};

class Center : public Robot {
  private:
    uint8_t motorPins[2]; // TODO: just use two separate variables for the pins named `armPin` and `clawPin`. set in constructor with `this.armPin` etc.
    MotorControl clawMotor, armMotor;

  public:
    Center(); 
    void action() override; //! robot subclass must override action
    void setServos(int armPin, int clawPin); // TODO: merge with constructor
    void clawControl(clawStatus reqStatus);
    void armControl(armStatus reqStatus);
};

/*
       ____   _____   _   _   _____   _____   ____
      / ___| | ____| | \ | | |_   _| | ____| |  _ \
     | |     |  _|   |  \| |   | |   |  _|   | |_) |
     | |___  | |___  | |\  |   | |   | |___  |  _ <
      \____| |_____| |_| \_|   |_|   |_____| |_| \_\
  
*/

/** Center Code
    --- Functions List ---
    Center
    clawControl - Based on what is inputed in main, opens, closes, or stops the claw. 
    armControl - Based on what is inputed in main, raises, lowers, or stops the arm. 
*/

/**
 * Description: Public function that starts the arm and claw motors and sets their starting status to stop. 
 * Author: @ n-johnson.3
 * Date: 9/19/22
**/
Center::Center() {

}

void Center::setServos(int armPin, int clawPin) {
  this->motorPins[0] = armPin, this->motorPins[1] = clawPin;
  armMotor.attach(armPin), clawMotor.attach(clawPin);
}

void Center::action() {
  // Control the arm of the center
  if (ps5.Triangle()) {
    armControl(armStatus::Higher);
  } else if (ps5.Cross()) {
    armControl(armStatus::Lower);
  } else if (ps5.Circle()) {
    armControl(armStatus::Hold);
  } else {
    armControl(armStatus::Stop);
  }

  // Control the Claw of the center
  if (ps5.Up()) {
    clawControl(clawStatus::Open);
  } else if (ps5.Down()) {
    clawControl(clawStatus::Close);
  } else {
    clawControl(clawStatus::clawStop);
  }  
}

/**
 * Description: Public helper function that checks the claw status and updates the claw motor accordingly. 
 * Author: @ n-johnson.3
 * Date: 9/19/22
 **/
void Center::clawControl(clawStatus reqstatus) {
  if(reqstatus == clawStatus::Open) {
    clawMotor.write(0.1);
  } else if(reqstatus == clawStatus::Close) {
    clawMotor.write(-0.1);
  } else if(reqstatus == clawStatus::clawStop) {
    clawMotor.write(0);
  }
}

/**
 * Description: Public helper function that checks the arm status and updates the arm motor accordingly. 
 * Author: @ n-johnson.3
 * Date: 9/19/22
 **/
void Center::armControl(armStatus reqstatus) {
  if (reqstatus == armStatus::Lower) {
    armMotor.write(0.15);
  } else if (reqstatus == armStatus::Higher) {
    armMotor.write(-0.1);
  } else if (reqstatus == armStatus::Stop) {
    armMotor.write(0);
  } else if (reqstatus == armStatus::Hold) {
    armMotor.write(-0.05);
  }
}

#endif // OLD_CENTER_H