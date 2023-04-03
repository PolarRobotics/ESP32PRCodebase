#include <Arduino.h>
#include <Drive/Drive.h>
#include "DriveMecanum.h"

/**
 * @brief 
 * 
 * 
 * Motor Layout                                 Bottom-up view
 *                  ^                                  ^            
 *                  | Fwd                              | Fwd        
 *       _______________________            ______________________ 
 *      |   __             __   |          |                      |
 *      |  |LF|           |RF|  |          |  \\\            ///  |
 *      |  |1 |           |2 |  |          |  \\\            ///  |
 *      |  |__|           |__|  |          |  \\\            ///  |
 *      |           .           |          |          .           |
 *      |   __             __   |          |                      |
 *      |  |LR|           |RR|  |          |  ///            \\\  |
 *      |  |3 |           |4 |  |          |  ///            \\\  |
 *      |  |__|           |__|  |          |  ///            \\\  |
 *      |_______________________|          |______________________|
 * 
 */


DriveMecanum::DriveMecanum() {
    setMotorType(MOTORS::mecanummotor);
}

void DriveMecanum::setServos(uint8_t lfpin, uint8_t rfpin, uint8_t lrpin, uint8_t rrpin) {
    this->LF.attach(lfpin);
    this->RF.attach(rfpin);
    this->LR.attach(lrpin);
    this->RR.attach(rrpin);
}

void DriveMecanum::setStickPwr(int8_t leftX, int8_t leftY, int8_t rightX) {
    // normalize the 8-bit input to 1
    this->scaledLeftX  = leftX  / 127.5f;
    this->scaledLeftY  = leftY  / 127.5f;
    this->scaledRightX = rightX / 127.5f;
    // account for stick deadzone
    this->scaledLeftX  = fabs(scaledLeftX)  < STICK_DEADZONE ? 0 : scaledLeftX;
    this->scaledLeftY  = fabs(scaledLeftY)  < STICK_DEADZONE ? 0 : scaledLeftY;
    this->scaledRightX = fabs(scaledRightX) < STICK_DEADZONE ? 0 : scaledRightX;
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
void DriveMecanum::generateMotorValues() {
    // generate motion vector (strafe direction)
    this->r = hypot(scaledLeftX, scaledLeftY);
    this->theta = atan2(scaledLeftX, scaledLeftY);
    this->turnPwr = scaledRightX;

    // ensure motor power doesnt exceed 100%
    this->r = constrain(this->r, 0, 1);

    // calculate the x and y components from the stick input with an angle offset of 45 degrees
    // if the motor magnitude is zero, the angle would be undefined, so set the component value to zero
    this->x_comp = (r == 0) ? 0 : r * sin(theta + (PI/4));
    this->y_comp = (r == 0) ? 0 : r * cos(theta + (PI/4));

    // find the max value for each of the inputs, use to normalize the motor powers to 1
    // this->max = _max(_max(fabs(x_comp),fabs(y_comp)), _max(fabs(turnPwr),1));

    mmotorpwr[0] = (x_comp + turnPwr); // / max;
    mmotorpwr[1] = (y_comp - turnPwr); // / max;
    mmotorpwr[2] = (y_comp + turnPwr); // / max;
    mmotorpwr[3] = (x_comp - turnPwr); // / max;

    // setMotorPwr(r * x_comp / max + turnPwr, 0);
    // setMotorPwr(r * y_comp / max - turnPwr, 1);
    // setMotorPwr(r * y_comp / max + turnPwr, 2);
    // setMotorPwr(r * x_comp / max - turnPwr, 3);

    // float tempbsn = getBSN();

    // scale the motor power, so it doesnt exceed 1, which would be bad
    // if ((r + abs(turnPwr)) > tempbsn) {
    //     setMotorPwr(getMotorPwr(0) / (r + abs(turnPwr)), 0);
    //     setMotorPwr(getMotorPwr(1) / (r + abs(turnPwr)), 1);
    //     setMotorPwr(getMotorPwr(2) / (r + abs(turnPwr)), 2);
    //     setMotorPwr(getMotorPwr(3) / (r + abs(turnPwr)), 3);
    // }

}

void DriveMecanum::emergencyStop() {
    this->LF.writelow();
    this->RF.writelow();
    this->LR.writelow();
    this->RR.writelow();
}

void DriveMecanum::update() {
    generateMotorValues();

    // ramp may need removed, including for testing purposes
    // setMotorPwr(ramp(getMotorPwr(0), 0), 0);
    // setMotorPwr(ramp(getMotorPwr(1), 1), 1);
    // setMotorPwr(ramp(getMotorPwr(2), 2), 2);
    // setMotorPwr(ramp(getMotorPwr(3), 3), 3);

    this->LF.write(mmotorpwr[0]); // getMotorPwr(0)
    this->RF.write(mmotorpwr[1]); // getMotorPwr(1)
    this->LR.write(mmotorpwr[2]); // getMotorPwr(2)
    this->RR.write(mmotorpwr[3]); // getMotorPwr(3)
}

void DriveMecanum::drift() {
    this->update(); // if drift is called, just call update
    // in the future we could change this to allow for left and right pivots
    // pivot around a point between the two left and two right wheels
}

void DriveMecanum::printDebugInfo() {
    Serial.print(F("LIX: "));
    Serial.print(scaledLeftX);
    Serial.print(F("  LIY: "));
    Serial.print(scaledLeftY);
    Serial.print(F("  RIX: "));
    Serial.print(scaledRightX);

    Serial.print(F("  Xcomp: "));
    Serial.print(this->x_comp);
    Serial.print(F("  Ycomp: "));
    Serial.print(this->y_comp);

    Serial.print(F("  PWRO: "));
    for (int i = 0; i < NUM_MOTORS; i++) {
        Serial.print(i);
        Serial.print(F("  "));
        Serial.print(getMotorPwr(i));
        Serial.print(F("  "));
    }
    Serial.print(F("\n"));
}
