#include "Drive/Drive.h"
#include <Arduino.h>
#include "Drive.h"

/**
 * @brief Drive Class, base class for specialized drive classes, this configuration is intended for the standard linemen.
 * this class takes the the the stick input, scales the turning value for each motor and ramps that value over time,
 * then sets the ramped value to the motors
 * @authors Rhys Davies (@rdavies02), Max Phillips (@RyzenFromFire)
 *
 * @class
 *    2 motor configuration shown below
 *
 *               ^
 *               | Fwd
 *       _________________
 *      |        _        |
 *      |       |O|       |       O: represents the Omniwheel, a wheel that can turn on 2 axis
 *      |       |_|       |       L: represents the left Wheel, powered by the left motor via a chain
 *      |  _           _  |            - the left motor would turn ccw to move the bot forward
 *      | |L|         |R| |       R: represents the right Wheel, powered by the right motor via a chain
 *      | |_|         |_| |            - the right motor would turn cw to move the bot forward
 *      |_________________|
 *
 * @todo
 *  - add a turning radius parameter, needed for the kicker
 *  - add mechanium driving code, for the new center, needed next semester (Spring 2023)
 *
 * Default configuration:
 * @param leftmotorpin the arduino pin needed for the left motor, needed for servo
 * @param rightmotorpin the arduino pin needed for the right motor, needed for servo
*/
Drive::Drive() {
    this->motorType = MOTORS::big; // default to long motors
}
void Drive::setServos(uint8_t lpin, uint8_t rpin){
    motorPins[0] = lpin, motorPins[1] = rpin;
    ledcAttachPin(lpin, M1_PWMCH);
    ledcAttachPin(rpin, M2_PWMCH);
    ledcSetup(M1_PWMCH, PWM_FREQ, PWM_RES);
    ledcSetup(M2_PWMCH, PWM_FREQ, PWM_RES);
};

void Drive::setMotorType(MOTORS motorType) {
    this->motorType = motorType;
}


/**
 * setStickPwr takes the stick values passed in and normalizes them to values between -1 and 1
 * and sets this value to the private variables stickFwdRev and stickTurn respectively
 * @author Rhys Davies
 * Created: 9-12-2022
 *
 * @param leftY the forward backward value from the left stick an unsigned 8-bit float (0 to 255)
 * @param rightX the left right value from the right stick an unsigned 8-bit float (0 to 255)
*/
void Drive::setStickPwr(int8_t leftY, int8_t rightX) {
    // left stick all the way foreward is 0, backward is 255
    // +: forward, -: backward. needs to be negated so that forward is forward and v.v. subtracting 1 bumps into correct range
    // stickForwardRev = (0 - (leftY / 127.5 - 1)); 
    // stickTurn = (rightX / 127.5 - 1); // +: right turn, -: left turn. subtracting 1 bumps into correct range
    stickForwardRev = (leftY / (float)127.5);
    stickTurn = (rightX / (float)127.5);  

    // stick deadzones
    // set to zero (no input) if within the set deadzone
    if (fabs(stickForwardRev) < STICK_DEADZONE) {
      stickForwardRev = 0;
    }
    if (fabs(stickTurn) < STICK_DEADZONE) {
      stickTurn = 0;
    }
    // Ensure the stick values do not go above 1 or below -1
    // stickForwardRev = constrain(stickForwardRev, -1, 1);
    // stickTurn = constrain(stickTurn, -1, 1);
}


/**
 * @brief setBSN sets the internal variable to the requested percent power, this is what the motor power gets multiplied by,
 * this is where the boost, slow and normal scalars get passed in
 * @author Rhys Davies
 * Created: 9-12-2022
 *
 * @param bsn input speed choice Drive::Boost, Drive::Slow, Drive::Normal
*/
void Drive::setBSN(SPEED bsn) {
    // set the scalar to zero if the requested value is greater than 1, this is not entirely necessary, but is a safety
    // BSNscalar = (powerMultiplier > 1) ? 0 : powerMultiplier;
    switch (bsn) {
        case boost: {
            if (motorType == MOTORS::big) BSNscalar = BIG_BOOST_PCT;
            else /* motorType == MOTORS::small */ BSNscalar = SMALL_BOOST_PCT;
            break;
        }
        case normal: {
            if (motorType == MOTORS::big) BSNscalar = BIG_NORMAL_PCT;
            else /* motorType == MOTORS::small */ BSNscalar = SMALL_NORMAL_PCT;
            break;
        }
        case slow: {
            if (motorType == MOTORS::big) BSNscalar = BIG_SLOW_PCT;
            else /* motorType == MOTORS::small */ BSNscalar = SMALL_SLOW_PCT;
            break;
        }
        case brake: {
            BSNscalar = BRAKE_BUTTON_PCT;
            break;
        }
    }
}


/**
 * generateTurnScalar takes the input stick power and scales the max turning power allowed with the forward power input
 * @authors Grant Brautigam, Rhys Davies, Max Phillips
 * Created: 9-12-2022
*/
void Drive::generateMotionValues() {
    if (fabs(stickForwardRev) < STICK_DEADZONE) { // fwd stick is zero
        if (fabs(stickTurn) < STICK_DEADZONE) { // turn stick is zero
            motorPower[0] = 0, motorPower[1] = 0; // not moving, set motors to zero
        } else if (stickTurn > STICK_DEADZONE) { // turning right, but not moving forward much so use tank mode
            motorPower[0] = BSNscalar * abs(stickTurn)  * TANK_MODE_PCT;
            motorPower[1] = -BSNscalar * abs(stickTurn) * TANK_MODE_PCT;
        } else if (stickTurn < -STICK_DEADZONE) { // turning left, but not moving forward muchso use tank mode
            motorPower[0] = -BSNscalar * abs(stickTurn) * TANK_MODE_PCT;
            motorPower[1] = BSNscalar * abs(stickTurn)  * TANK_MODE_PCT;
        } // no general else since encountered infinite loop
    } else { // fwd stick is not zero
        if (fabs(stickTurn) < STICK_DEADZONE) { // turn stick is zero
            // just move forward directly
            motorPower[0] = BSNscalar * stickForwardRev;
            motorPower[1] = BSNscalar * stickForwardRev;
        } else { // moving forward and turning
            /*
            if the sticks are not in any of the edge cases tested for above (when both sticks are not 0),
            a value must be calculated to determine how to scale the motor that is doing the turning.
            i.e.: if the user moves the left stick all the way forward (stickFwdRev = 1), and they are attempting
            to turn right. The left motor should get set to 1 and the right motor should get set to
            some value less than 1, this value is determined by the function calcTurningMotorValue
            */
            if(stickTurn > STICK_DEADZONE) { // turn Right
                switch(abs((BSNscalar * stickForwardRev)) > abs(lastRampPower[0])) {
                    case true: calcTurningMotorValues(stickTurn, abs(lastRampPower[0]), 1); break;
                    case false: calcTurningMotorValues(stickTurn, abs((BSNscalar * stickForwardRev)), 1); break;
                }
                //calcTurningMotorValues(stickTurn, lastRampPower[0], 1);
                motorPower[0] = copysign(turnMotorValues[0], stickForwardRev);
                motorPower[1] = copysign(turnMotorValues[1], stickForwardRev);
            } else if(stickTurn < -STICK_DEADZONE) { // turn Left
                switch(abs((BSNscalar * stickForwardRev)) > abs(lastRampPower[1])) {
                    case true: calcTurningMotorValues(stickTurn, abs(lastRampPower[1]), 0); break;
                    case false: calcTurningMotorValues(stickTurn, abs((BSNscalar * stickForwardRev)), 0); break;
                }
                //calcTurningMotorValues(stickTurn, lastRampPower[1], 0);
                motorPower[0] = copysign(turnMotorValues[1], stickForwardRev);
                motorPower[1] = copysign(turnMotorValues[0], stickForwardRev);
            }
        }
    }
}


/**
 * @brief Work in progress! as of 11/27 stuffs changin' calcTurningMotorValue generates a value to be set to the turning motor, the motor that corresponds to the direction of travel
 * @authors Grant Brautigam, Rhys Davies
 * Created: 9-12-2022
 * updated: 11-27-22
 * Mathematical model:
 *  NOT REALLY TRUE ANYMORE
 *  TurningMotor = TurnStickNumber(1-offset)(CurrentPwrFwd)^2+(1-TurnStickNumber)*CurrentPwrFwd
 *   *Note: CurrentPwrFwd is the current power, not the power from the stick
 *
 * @param stickTrn the absoulte value of the current turning stick input
 * @param prevPwr the non-turning motor value from the previous loop, which was actually sent to the motor
 * @return float - the value to get set to the turning motor (the result of the function mention above)
 */
void Drive::calcTurningMotorValues(float stickTrn, float prevPwr, int dir) {
    
    float MaxTurnDiff = (turnMin - turnMax) * (abs(prevPwr)) + turnMax;
    float TurnDifference = abs(stickTrn)*MaxTurnDiff;

    if ((prevPwr-TurnDifference) <= 0){
        turnMotorValues[0] = prevPwr + abs(prevPwr-TurnDifference);
        turnMotorValues[1] = 0;
    } else {
        turnMotorValues[0] = prevPwr;
        turnMotorValues[1] = prevPwr-TurnDifference;
    }

    lastTurnPwr = TurnDifference;
}


/**
 * @brief ramp slowly increases the motor power each iteration of the main loop,
 * the period and amount of increase is determined by the constants TIME_INCREMENT and ACCELERATION_RATE
 * this function is critical in ensuring the bot has proper traction with the floor,
 * the smaller ACCELERATION_RATE or larger TIME_INCREMENT is, the slower the ramp will be,
 * think of it as the slope y=mx+b
 *
 * FUTURE: combine ACCELERATION_RATE and TIME_INCREMENT into one constant,
 * to allow for better tuning of ramp, we ran into problems during the 2022 comp with this,
 * possibly finding the best values for certain surfaces and storing them into a table and pulling from
 * this to determine a comfortable range for the drivers.
 *
 * @authors Max Phillips, Grant Brautigam
 * Created: early 2022
 *
 * @param requestedPower
 * @param mtr pass 0 for left and 1 for right, used to help ease with storing values for multiple motors
 * @return float
 */
float Drive::ramp(float requestedPower, uint8_t mtr) {

    if (millis() - lastRampTime[mtr] >= TIME_INCREMENT) {
        if (abs(requestedPower) < THRESHOLD) { // if the input is effectively zero
        // Experimental Braking Code
            if (abs(currentPower[mtr]) < 0.1) { // if the current power is very small just set it to zero
                currentPower[mtr] = 0;
            }
            else {
                currentPower[mtr] *= BRAKE_PERCENTAGE;
            }
            //currentPower[mtr] = 0;
            lastRampTime[mtr] = millis();
        }
        else if (abs(requestedPower - currentPower[mtr]) < ACCELERATION_RATE) { // if the input is effectively at the current power
            return requestedPower;
        }
        // if we need to increase speed and we are going forward
        else if (requestedPower > currentPower[mtr] && requestedPower > 0) { 
            currentPower[mtr] = currentPower[mtr] + ACCELERATION_RATE;
            lastRampTime[mtr] = millis();
        }
        // if we need to decrease speed and we are going forward
        else if (requestedPower < currentPower[mtr] && requestedPower > 0) { 
            currentPower[mtr] = currentPower[mtr] - ACCELERATION_RATE;
            lastRampTime[mtr] = millis();
        }
        // if we need to increase speed and we are going in reverse
        else if (requestedPower < currentPower[mtr] && requestedPower < 0) { 
            currentPower[mtr] = currentPower[mtr] - ACCELERATION_RATE;
            lastRampTime[mtr] = millis();
        }
        // if we need to decrease speed and we are going in reverse
        else if (requestedPower > currentPower[mtr] && requestedPower < 0) { 
            currentPower[mtr] = currentPower[mtr] + ACCELERATION_RATE;
            lastRampTime[mtr] = millis();
        }
    }

    return currentPower[mtr];
}


/**
 * normalizes the signed power value from the ramp function to an unsigned value that the servo function can take
 * @authors Rhys Davies, Grant Brautigam, Alex Brown
 * Updated: 9-13-2022
 *
 * @param rampPwr the value to be normalized. Hopefully a value between [-1, 1]
 * @return normalized PWM value (between 1000 to 2000)
*/
uint32_t Drive::Convert2PWM(float rampPwr) {
    return (rampPwr + 1) * 500 + 1000;
}

uint16_t Drive::convert2Duty(uint32_t timeon_us) {
    // convert the time on to a duty cycle, then scale it to a 16-bit number
    return (timeon_us / (PWM_PERIOD * 1000)) * (pow(2, PWM_RES) / 1000);
}

/**
 * alternate for servos writeMicroseconds, a function to set the motors based on a power input (-1 to 1),
 * manually sets the motor high for the pwm time.
 * @author Rhys Davies
 * Created: 10-5-2022
 * Updated: 10-29-2022
 *
 * Set the motors by outputting a pulse with a period corresponding to the motor power,
 * determined by Convert2PWMVal. These calculations can not be put into a function,
 * because it confuses the compiler as these are time critical tasks.
 * 
 * FUTURE: write this so both pins are HIGH at the same time, and the one that goes low first is called,
 * because the beginning of the pulse is at the same time for both pins, but this isnt entirely necessary
 * considering both motors operate independently on the sabertooth
 *
 * @param pwr the motor power to be set
 * @param pin the motor to be set (0 for left, 1 for right)
*/
void Drive::setMotorPWM(float pwr, byte pin) {
    digitalWrite(motorPins[pin], HIGH);
    delayMicroseconds(Convert2PWM(pwr) - 40);
    digitalWrite(motorPins[pin], LOW);
    delayMicroseconds(2000 - Convert2PWM(pwr) - 40); //-170
    // digitalWrite(motorPins[0], HIGH);
    // delayMicroseconds(Convert2PWMVal(motorPower[0]) - 40);
    // digitalWrite(motorPins[0], LOW);
    // // delayMicroseconds(2000 - Convert2PWMVal(motorPower[0]) - 40); //-170
    // digitalWrite(motorPins[1], HIGH);
    // delayMicroseconds(Convert2PWMVal(motorPower[1]) - 40);
    // digitalWrite(motorPins[1], LOW);
    // delayMicroseconds(2000 - Convert2PWMVal(motorPower[1]) - 40); //-170
}

/**
 * returns the stored motor value in the class
 * @param mtr the motor number to get, an array index, so 0 -> mtr 1, etc...
 * @return returns the stored motor power for a given motor
*/
float Drive::getMotorPwr(uint8_t mtr) {
    return motorPower[mtr];
}

void Drive::emergencyStop() {
    ledcWrite(M1_PWMCH, convert2Duty(Convert2PWM(0)));
    ledcWrite(M2_PWMCH, convert2Duty(Convert2PWM(0)));
}

/**
 * prints the internal variables to the serial monitor in a clean format,
 * this function exists out of pure laziness to not have to comment out all the print statments
 * @author
 * Updated:
*/
void Drive::printDebugInfo() {
    Serial.print(F("L_Hat_Y: "));
    Serial.print(stickForwardRev);
    Serial.print(F("  R_HAT_X: "));
    Serial.print(stickTurn);

    Serial.print(F("  |  Turn: "));
    Serial.print(lastTurnPwr);

    // Serial.print(F("  |  Left ReqPwr: "));
    // Serial.print(motorPower[0]);
    // Serial.print(F("  Right ReqPwr: "));
    // Serial.print(motorPower[1]);
    
    // Serial.print(F("  lastRampTime "));
    // Serial.print(lastRampTime[0]);
    // Serial.print(F("  requestedPower "));
    // Serial.print(requestedPower);
    // Serial.print(F("  current "));
    // Serial.print(currentPower[0]);
    // Serial.print(F("  requestedPower - currentPower "));
    // Serial.println(requestedPower - currentPower[mtr], 10);

    Serial.print(F("  Left Motor: "));
    Serial.print(motorPower[0]);
    Serial.print(F("  Right: "));
    Serial.print(motorPower[1]);

    Serial.print(F("  |  Left Motor: "));
    Serial.print(Convert2PWM(-motorPower[0]));
    Serial.print(F("  Right: "));
    Serial.println(Convert2PWM(motorPower[1]));

    // Serial.print(F("  |  Left Motor: "));
    // Serial.print(convert2Duty(Convert2PWM(-motorPower[0])));
    // Serial.print(F("  Right: "));
    // Serial.println(convert2Duty(Convert2PWM(motorPower[1])));

}

/**
 * @brief updates the motors after calling all the functions to generate
 * turning and scaling motor values, the intention of this is so the
 * programmer doesnt have to call all the functions, this just handles it,
 * reducing clutter in the main file.
 * DO NOT CALL THIS FUNCTION UNTIL setStickPwr and setBSN have been called before update
 * @author Rhys Davies
 * Created: 9-12-2022
 * Updated: 10-11-2020
*/
void Drive::update() {
    // Generate turning motion
    generateMotionValues();

    // get the ramp value
    motorPower[0] = ramp(motorPower[0], 0);
    motorPower[1] = ramp(motorPower[1], 1);

    // Set the ramp value to a function, needed for generateMotionValues
    lastRampPower[0] = motorPower[0];
    lastRampPower[1] = motorPower[1];
    
    ledcWrite(M1_PWMCH, convert2Duty(Convert2PWM(motorPower[0])));
    ledcWrite(M2_PWMCH, convert2Duty(Convert2PWM(motorPower[1])));
}

/**
 * @brief updates the motors after calling all the functions to generate
 * turning and scaling motor values, the intention of this is so the
 * programmer doesnt have to call all the functions, this just handles it,
 * reducing clutter in the main file.
 * @author Rhys Davies
 * Created: 9-12-2022
 * Updated: 10-11-2020
*/
void Drive::drift() {
    if (stickTurn > STICK_DEADZONE) { // turning right, but not moving forward much so use tank mode
        motorPower[0] = BSNscalar * abs(stickTurn)  * DRIFT_MODE_PCT;
        motorPower[1] = 0;
    } else if (stickTurn < -STICK_DEADZONE) { // turning left, but not moving forward much so use tank mode
        motorPower[0] = 0;
        motorPower[1] = BSNscalar * abs(stickTurn)  * DRIFT_MODE_PCT;
    } else {
        motorPower[0] = 0;
        motorPower[1] = 0;
    }

    ledcWrite(M1_PWMCH, convert2Duty(Convert2PWM(motorPower[0])));
    ledcWrite(M2_PWMCH, convert2Duty(Convert2PWM(motorPower[1])));
}
