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
#include <Robot/Center.h>


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

/**
 * Description: Public helper function that checks the claw status and updates the claw motor accordingly. 
 * Author: @ n-johnson.3
 * Date: 9/19/22
 **/
void Center::clawControl(clawStatus reqstatus) {
  if(reqstatus == clawStatus::Open) {
    clawMotor.write(0.05);
  } else if(reqstatus == clawStatus::Close) {
    clawMotor.write(-0.05);
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
    armMotor.write(0.05);
  } else if (reqstatus == armStatus::Higher) {
    armMotor.write(-0.05);
  } else if (reqstatus == armStatus::Stop) {
    armMotor.write(0);
  } else if (reqstatus == armStatus::Hold) {
    armMotor.write(-0.025);
  }

}