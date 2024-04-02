#include <Arduino.h>
#include <Drive/Drive.h>
#include "DriveMecanum.h"

/**
 * @brief 
 * 
 * 
 *             Motor Layout                      Bottom-up view
 *
 *
 *                  ^                                  ^            
 *                  | Fwd                              | Fwd        
 *       _______________________            ______________________ 
 *      |   __             __   |          |                      |
 *      |  |LF|           |RF|  |          |  \\\            ///  |
 *      |  |1 |           |2 |  |          |  \\\            ///  |
 *      |  |__|           |__|  |          |  \\\            ///  |
 *      |           .           |          |          .           |
 *      |   __             __   |          |                      |
 *      |  |LB|           |RB|  |          |  ///            \\\  |
 *      |  |3 |           |4 |  |          |  ///            \\\  |
 *      |  |__|           |__|  |          |  ///            \\\  |
 *      |_______________________|          |______________________|
 * 
 */

//! Must call base class constructor with appropriate arguments
DriveMecanum::DriveMecanum() : Drive(BotType::mecanum_center, MotorType::mecanum) {
  // initialize array
  for (int i = 0; i < MC_NUM_MOTORS; i++) {
    mecanumMotorPwr[i] = 0.0f;
  }
}

void DriveMecanum::setupMotors(uint8_t pinLF, uint8_t pinRF, uint8_t pinLB, uint8_t pinRB) {
    this->LF.setup(pinLF);
    this->RF.setup(pinRF);
    this->LB.setup(pinLB);
    this->RB.setup(pinRB);
}

void DriveMecanum::setStickPwr(int8_t leftX, int8_t leftY, int8_t rightX) {
    // normalize the 8-bit input to 1
    this->stickForward = leftY  / 127.5f;
    this->stickStrafe  = leftX  / 127.5f;
    this->stickTurn    = rightX / 127.5f;

    // account for stick deadzone
    this->stickForward = fabs(stickForward) < MC_STICK_DEADZONE ? 0 : stickForward;
    this->stickStrafe  = fabs(stickStrafe)  < MC_STICK_DEADZONE ? 0 : stickStrafe;
    this->stickTurn    = fabs(stickTurn)    < MC_STICK_DEADZONE ? 0 : stickTurn;
}


/**
 * @brief generateMotorValues
 * 
 * Research links: 
 * - https://www.youtube.com/watch?v=gnSW2QpkGXQ
 * - https://robotics.stackexchange.com/questions/20088/how-to-drive-mecanum-wheels-robot-code-or-algorithm
 * - https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html
 * Drive directions cheat sheet: https://gm0.org/en/latest/_images/mecanum-drive-directions.png 
 * 
 * Current model in desmos:
 * https://www.desmos.com/calculator/xoqso2wmiw
 */
void DriveMecanum::generateMotorValuesOld() {
    // generate motion vector (strafe direction)
    // might want to move these, because ramp may need to be called before these
    // we may want to ramp the magnitude and turnPwr, instead of individual motors
    this->r = hypot(stickStrafe, stickForward);
    this->theta = atan2(stickStrafe, stickForward);
    this->turnPwr = stickTurn;

    // ensure motor power doesnt exceed 100%
    this->r = constrain(this->r, 0, 1);

    // this->x_comp = sin(theta + (PI/4));
    // this->y_comp = cos(theta + (PI/4));

    this->x_comp = (r == 0) ? 0 : r * sin(theta + (PI/4));
    this->y_comp = (r == 0) ? 0 : r * cos(theta + (PI/4));

    // this->max = _max(x_comp, y_comp);
    // motorPwr[0] = r * cos(theta) + turnPwr;
    // motorPwr[1] = r * sin(theta) - turnPwr;
    // motorPwr[2] = r * sin(theta) + turnPwr;
    // motorPwr[3] = r * cos(theta) - turnPwr;

    /*
        mecanum edge cases:
            theta is undefined when X and Y are both zero

        not even sure if a mecanum bot can strafe at 45 degrees
        https://robotics.stackexchange.com/questions/20088/how-to-drive-mecanum-wheels-robot-code-or-algorithm
    */

    // setReqMotorPwr(r * x_comp + turnPwr, 0);
    // setReqMotorPwr(r * y_comp - turnPwr, 1);
    // setReqMotorPwr(r * y_comp + turnPwr, 2);
    // setReqMotorPwr(r * x_comp - turnPwr, 3);

    
    // mecanumMotorPwr[0] = r * x_comp + turnPwr;
    // mecanumMotorPwr[1] = r * y_comp - turnPwr;
    // mecanumMotorPwr[2] = r * y_comp + turnPwr;
    // mecanumMotorPwr[3] = r * x_comp - turnPwr;

    mecanumMotorPwr[0] = x_comp + turnPwr;
    mecanumMotorPwr[1] = y_comp - turnPwr;
    mecanumMotorPwr[2] = y_comp + turnPwr;
    mecanumMotorPwr[3] = x_comp - turnPwr;


    // setReqMotorPwr(r * x_comp / max + turnPwr, 0);
    // setReqMotorPwr(r * y_comp / max - turnPwr, 1);
    // setReqMotorPwr(r * y_comp / max + turnPwr, 2);
    // setReqMotorPwr(r * x_comp / max - turnPwr, 3);

    // float tempbsn = getBSN();

    // scale the motor power, so it doesnt exceed 1, which would be bad
    // if ((r + abs(turnPwr)) > tempbsn) {
    //     setReqMotorPwr(getReqMotorPwr(0) / (r + abs(turnPwr)), 0);
    //     setReqMotorPwr(getReqMotorPwr(1) / (r + abs(turnPwr)), 1);
    //     setReqMotorPwr(getReqMotorPwr(2) / (r + abs(turnPwr)), 2);
    //     setReqMotorPwr(getReqMotorPwr(3) / (r + abs(turnPwr)), 3);
    // }

}

void DriveMecanum::generateMotorValues() {

    mecanumMotorPwr[0] = stickForward + stickStrafe + stickTurn; // LF
    mecanumMotorPwr[1] = stickForward - stickStrafe - stickTurn; // RF
    mecanumMotorPwr[2] = stickForward - stickStrafe + stickTurn; // LB
    mecanumMotorPwr[3] = stickForward + stickStrafe - stickTurn; // RB
}



void DriveMecanum::emergencyStop() {
    this->LF.write(0);
    this->RF.write(0);
    this->LB.write(0);
    this->RB.write(0);
}

void DriveMecanum::update() {
    generateMotorValues();

    // ramp may need removed, including for testing purposes
    // setReqMotorPwr(ramp(getReqMotorPwr(0), 0), 0);
    // setReqMotorPwr(ramp(getReqMotorPwr(1), 1), 1);
    // setReqMotorPwr(ramp(getReqMotorPwr(2), 2), 2);
    // setReqMotorPwr(ramp(getReqMotorPwr(3), 3), 3);

    this->LF.write(mecanumMotorPwr[0]); // getReqMotorPwr(0)
    this->RF.write(mecanumMotorPwr[1]); // getReqMotorPwr(1)
    this->LB.write(mecanumMotorPwr[2]); // getReqMotorPwr(2)
    this->RB.write(mecanumMotorPwr[3]); // getReqMotorPwr(3)
}

void DriveMecanum::printDebugInfo() {
    Serial.print(F("SLX: "));
    Serial.print(stickStrafe);
    Serial.print(F("  SLY: "));
    Serial.print(stickForward);
    Serial.print(F("  SRX: "));
    Serial.print(stickTurn);

    Serial.print(F("  LF: "));
    Serial.print(mecanumMotorPwr[0]);
    Serial.print(F("  RF: "));
    Serial.print(mecanumMotorPwr[1]);
    Serial.print(F("  LB: "));
    Serial.print(mecanumMotorPwr[2]);
    Serial.print(F("  RB: "));
    Serial.print(mecanumMotorPwr[3]);

    Serial.print(F("\n"));
}
