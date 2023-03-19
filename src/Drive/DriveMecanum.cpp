#include <Arduino.h>
#include <Drive/Drive.h>
#include "DriveMecanum.h"

/**
 * @brief 
 * 
 * 
 * Motor Layout
 *                  ^
 *                  | Fwd
 *       _______________________
 *      |   __             __   |
 *      |  |LF|           |RF|  |
 *      |  |1 |           |2 |  |
 *      |  |__|           |__|  |
 *      |           .           |
 *      |   __             __   |
 *      |  |LR|           |RR|  |
 *      |  |3 |           |4 |  |
 *      |  |__|           |__|  |
 *      |_______________________|
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
    this->scaledLeftX  = leftX  / (float)127.5;
    this->scaledLeftY  = leftY  / (float)127.5;
    this->scaledRightX = rightX / (float)127.5;
    // account for stick deadzone
    this->scaledLeftX  = fabs(scaledLeftX)  < STICK_DEADZONE ? 0 : scaledLeftX;
    this->scaledLeftY  = fabs(scaledLeftY)  < STICK_DEADZONE ? 0 : scaledLeftY;
    this->scaledRightX = fabs(scaledRightX) < STICK_DEADZONE ? 0 : scaledRightX;
}


/**
 * @brief generateMotorValues
 * 
 * referenced from: 
 * - https://www.youtube.com/watch?v=gnSW2QpkGXQ
 * 
 */
void DriveMecanum::generateMotorValues() {
    // generate motion vector (strafe direction)
    // might want to move these, because ramp may need to be called before these
    // we may want to ramp the magnitude and turnPwr, instead of individual motors
    this->r = hypot(scaledLeftX, scaledLeftY);
    this->theta = atan2(scaledLeftX, scaledLeftY);
    this->turnPwr = scaledRightX;

    this->x_comp = cos(theta - (PI/4));
    this->y_comp = sin(theta - (PI/4));
    this->max = _max(x_comp, y_comp);
    // motorPwr[0] = r * cos(theta) + turnPwr;
    // motorPwr[1] = r * sin(theta) - turnPwr;
    // motorPwr[2] = r * sin(theta) + turnPwr;
    // motorPwr[3] = r * cos(theta) - turnPwr;

    setMotorPwr(r * x_comp / max + turnPwr, 0);
    setMotorPwr(r * y_comp / max - turnPwr, 1);
    setMotorPwr(r * y_comp / max + turnPwr, 2);
    setMotorPwr(r * x_comp / max - turnPwr, 3);

    float tempbsn = getBSN();

    // scale the motor power, so it doesnt exceed 1, which would be bad
    if ((r + abs(turnPwr)) > tempbsn) {
        setMotorPwr(getMotorPwr(0) / (r + abs(turnPwr)), 0);
        setMotorPwr(getMotorPwr(1) / (r + abs(turnPwr)), 1);
        setMotorPwr(getMotorPwr(2) / (r + abs(turnPwr)), 2);
        setMotorPwr(getMotorPwr(3) / (r + abs(turnPwr)), 3);
    }

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
    setMotorPwr(ramp(getMotorPwr(0), 0), 0);
    setMotorPwr(ramp(getMotorPwr(1), 1), 1);
    setMotorPwr(ramp(getMotorPwr(2), 2), 2);
    setMotorPwr(ramp(getMotorPwr(3), 3), 3);

    this->LF.write(getMotorPwr(0));
    this->RF.write(getMotorPwr(1));
    this->LR.write(getMotorPwr(2));
    this->RR.write(getMotorPwr(3));
}

void DriveMecanum::drift() {
    this->update(); // if drift is called, just call update
}

void DriveMecanum::printDebugInfo() {
    Serial.print(F("L_Hat_X: "));
    Serial.print(scaledLeftX);
    Serial.print(F("  L_Hat_Y: "));
    Serial.print(scaledLeftY);
    Serial.print(F("  R_HAT_X: "));
    Serial.print(scaledRightX);

    Serial.print(F("  Output motor powers: "));
    for (int i = 0; i < NUM_MOTORS; i++) {
        Serial.print(i);
        Serial.print(F("  "));
        Serial.print(getMotorPwr(i));
        Serial.print(F("  "));
    }
}
