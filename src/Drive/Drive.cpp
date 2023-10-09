#include <Arduino.h>
#include "Drive/Drive.h"
#include "Robot/MotorControl.h"

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
  Drive(lineman, big_ampflow);
}

Drive::Drive(BotType botType, MotorType motorType, float gearRatio) {
  this->botType = botType;
  this->motorType = motorType;

  if (botType == quarterback) {
    this->BIG_BOOST_PCT = 0.8; 
    this->BIG_NORMAL_PCT = 0.4; 
    this->BIG_SLOW_PCT = 0.3;
  } else {
    this->BIG_BOOST_PCT = 0.7;  
    this->BIG_NORMAL_PCT = 0.6; 
    this->BIG_SLOW_PCT = 0.3;
  }

  // initialize arrays
  for (int i = 0; i < NUM_MOTORS; i++) {
    requestedMotorPower[i] = 0.0f;
    currentRampPower[i] = 0.0f;
    lastRampPower[i] = 0.0f;
    turnMotorValues[i] = 0.0f;
  }

  if (botType != mecanum_center) {
    // initialize parameters for turning model
    wheelBase = 9.75;
    omega = 0;
    omega_L = 0;
    omega_R = 0;
    R = 0;
    R_Max = 24;
    R_Min = wheelBase/2;
    max_RPM = 4000;
    min_RPM = 200;
  } 
}

void Drive::setServos(uint8_t lpin, uint8_t rpin) {
    //this->motorPins[0] = lpin, this->motorPins[1] = rpin;
    this->M1 = new MotorControl(motorType, false, this->gearRatio);
    this->M2 = new MotorControl(motorType, false, this->gearRatio);

    M1->setup(lpin), M2->setup(rpin);
}

/**
 * setServos
 * @brief to be called when setting up a motor with an encoder
 * 
 * 
*/
void Drive::setServos(uint8_t lpin, uint8_t rpin, uint8_t left_enc_a_pin, uint8_t left_enc_b_pin, uint8_t right_enc_a_pin, uint8_t right_enc_b_pin) {
    //this->motorPins[0] = lpin, this->motorPins[1] = rpin;
    this->M1 = new MotorControl(motorType, true, this->gearRatio);
    this->M2 = new MotorControl(motorType, true, this->gearRatio);
    
    M1->setup(lpin, left_enc_a_pin, left_enc_b_pin);
    M2->setup(rpin, right_enc_a_pin, right_enc_b_pin);
}

void Drive::setMotorType(MotorType motorType) {
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
    stickForwardRev = (leftY / 127.5f);
    stickTurn = (rightX / 127.5f);  

    // stick deadzones
    // set to zero (no input) if within the set deadzone
    if (fabs(stickForwardRev) < STICK_DEADZONE)
      stickForwardRev = 0;
    
    if (fabs(stickTurn) < STICK_DEADZONE)
      stickTurn = 0;
    
}

float Drive::getForwardPower() {
    return stickForwardRev;
}

float Drive::getTurnPower() {
    return stickTurn;
}

/**
 * @brief setBSN sets the internal variable to the requested percent power, this is what the motor power gets multiplied by,
 * this is where the boost, slow and normal scalars get passed in
 * @author Rhys Davies
 * Created: 9-12-2022
 *
 * @param bsn input speed choice Drive::Boost, Drive::Slow, Drive::Normal
*/
void Drive::setBSN(Speed bsn) {
    // set the scalar to zero if the requested value is greater than 1, this is not entirely necessary, but is a safety
    switch (bsn) {
        case BOOST: {
            switch (motorType) {
                case MotorType::big_ampflow: { BSNscalar = BIG_BOOST_PCT; break; }
                case MotorType::small_ampflow: { BSNscalar = SMALL_BOOST_PCT; break; }
                case MotorType::mecanum: { BSNscalar = MECANUM_BOOST_PCT; break; }
                case MotorType::falcon: { BSNscalar = FALCON_BOOST_PCT; break; }
            }
            break;
        }
        case NORMAL: {
            switch (motorType) {
                case MotorType::big_ampflow: { BSNscalar = BIG_NORMAL_PCT; break; }
                case MotorType::small_ampflow: { BSNscalar = SMALL_NORMAL_PCT; break; }
                case MotorType::mecanum: { BSNscalar = MECANUM_NORMAL_PCT; break; }
                case MotorType::falcon: { BSNscalar = FALCON_NORMAL_PCT; break; }
            }
            break;
        }
        case SLOW: {
            switch (motorType) {
                case MotorType::big_ampflow: { BSNscalar = BIG_SLOW_PCT; break; }
                case MotorType::small_ampflow: { BSNscalar = SMALL_SLOW_PCT; break; }
                case MotorType::mecanum: { BSNscalar = MECANUM_SLOW_PCT; break; }
                case MotorType::falcon: { BSNscalar = FALCON_SLOW_PCT; break; }
            }
            break;
        }
        case BRAKE: {
            BSNscalar = BRAKE_BUTTON_PCT;
            break;
        }
    }
}

float Drive::getBSN() {
    return this->BSNscalar;
}

/**
 * generateTurnScalar takes the input stick power and scales the max turning power allowed with the forward power input
 * @authors Grant Brautigam, Rhys Davies, Max Phillips
 * Created: 9-12-2022
*/
void Drive::generateMotionValues() {
    if (fabs(stickForwardRev) < STICK_DEADZONE) { // fwd stick is zero
        if (fabs(stickTurn) < STICK_DEADZONE) { // turn stick is zero
            requestedMotorPower[0] = 0, requestedMotorPower[1] = 0; // not moving, set motors to zero
        } else if (stickTurn > STICK_DEADZONE) { // turning right, but not moving forward much so use tank mode
            requestedMotorPower[0] = BSNscalar * abs(stickTurn)  * TANK_MODE_PCT;
            requestedMotorPower[1] = -BSNscalar * abs(stickTurn) * TANK_MODE_PCT;
        } else if (stickTurn < -STICK_DEADZONE) { // turning left, but not moving forward muchso use tank mode
            requestedMotorPower[0] = -BSNscalar * abs(stickTurn) * TANK_MODE_PCT;
            requestedMotorPower[1] = BSNscalar * abs(stickTurn)  * TANK_MODE_PCT;
        } // no general else since encountered infinite loop
    } else { // fwd stick is not zero
        if (fabs(stickTurn) < STICK_DEADZONE) { // turn stick is zero
            // just move forward directly
            requestedMotorPower[0] = BSNscalar * stickForwardRev;
            requestedMotorPower[1] = BSNscalar * stickForwardRev;
        } else { // moving forward and turning
            /*
            if the sticks are not in any of the edge cases tested for above (when both sticks are not 0),
            a value must be calculated to determine how to scale the motor that is doing the turning.
            i.e.: if the user moves the left stick all the way forward (stickFwdRev = 1), and they are attempting
            to turn right. The left motor should get set to 1 and the right motor should get set to
            some value less than 1, this value is determined by the function calcTurningMotorValue
            */
            // if(stickTurn > STICK_DEADZONE) { // turn Right
            //     switch(abs((BSNscalar * stickForwardRev)) > abs(lastRampPower[0])) {
            //         case true: calcTurning(stickTurn, abs(lastRampPower[0]), 1); break;
            //         case false: calcTurning(stickTurn, abs(BSNscalar * stickForwardRev), 1); break;
            //     }
            //     //calcTurning(stickTurn, lastRampPower[0], 1);
            //     requestedMotorPower[0] = copysign(turnMotorValues[0], stickForwardRev);
            //     requestedMotorPower[1] = copysign(turnMotorValues[1], stickForwardRev);
            // } else if(stickTurn < -STICK_DEADZONE) { // turn Left
            //     switch(abs((BSNscalar * stickForwardRev)) > abs(lastRampPower[1])) {
            //         case true: calcTurning(stickTurn, abs(lastRampPower[1]), 0); break;
            //         case false: calcTurning(stickTurn, abs(BSNscalar * stickForwardRev), 0); break;
            //     }
            //     //calcTurning(stickTurn, lastRampPower[1], 0);
            //     requestedMotorPower[0] = copysign(turnMotorValues[1], stickForwardRev);
            //     requestedMotorPower[1] = copysign(turnMotorValues[0], stickForwardRev);
            // }

            calcTurning(stickTurn, fabs(BSNscalar * stickForwardRev));

            requestedMotorPower[0] = copysign(turnMotorValues[1], stickForwardRev);
            requestedMotorPower[1] = copysign(turnMotorValues[0], stickForwardRev);
        }
    }
}


/**
 * @brief calcTurning generates power values for each motor to achieve a given turn
 * @authors Grant Brautigam, Rhys Davies
 * Created: 9-12-2022
 * updated: 10-6-2023
 * Mathematical model:
 * Whitepaper: https://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf
 * 
 * Differential drive turning model
 * 
 * omega_R = (omega/R)*(R + l/2)
 * omega_L = (omega/R)*(R - l/2)
 *
 * omega_R: right wheel angular velocity
 * omega_R: left wheel angular velocity
 * omega: the rate of rotation around the ICC (Instantaneous Center of Curvature)
 * l: distance between each wheel (wheelbase) 
 * R: distance the ICC from the center of the wheelbase (l/2)
 * 
 * @param stickTrn the absoulte value of the current turning stick input
 * @param fwdLinPwr the non-turning motor value from the previous loop, which was actually sent to the motor
 */
void Drive::calcTurning(float stickTrn, float fwdLinPwr) {
    //R_Min = R_Min + abs(stickForwardRev)*(R_High_Min - R_Min); // start of turn scaling 

    // Calculate the R value from the stick turn input
    R = (1-abs(stickTrn))*(R_Max-R_Min) + R_Min;
    
    // calculate the requested angular velocity for the robot 
    omega = fwdLinPwr*max_RPM;
    
    // calculate the rpm for the left wheel
    omega_L = (omega/R)*(R+(wheelBase/2));
    // calculate the rpm for the right wheel
    omega_R = (omega/R)*(R-(wheelBase/2));

    // ensure the left wheel RPM doesnt go above the min or the max RPM
    omega_L = constrain(omega_L, min_RPM, max_RPM);
    // ensure the left wheel RPM doesnt go above the min or the max RPM
    omega_R = constrain(omega_R, min_RPM, max_RPM);

    turnMotorValues[0] = omega_L/max_RPM;
    turnMotorValues[1] = omega_R/max_RPM;
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
float Drive::ramp(float requestedPower, uint8_t mtr, float accelRate) {

    if (millis() - lastRampTime[mtr] >= TIME_INCREMENT) {
        if (abs(requestedPower) < THRESHOLD) { // if the input is effectively zero
        // Experimental Braking Code
            if (abs(currentRampPower[mtr]) < 0.1) { // if the current power is very small just set it to zero
                currentRampPower[mtr] = 0;
            }
            else {
                currentRampPower[mtr] *= BRAKE_PERCENTAGE;
            }
            //currentRampPower[mtr] = 0;
            lastRampTime[mtr] = millis();
        }
        else if (abs(requestedPower - currentRampPower[mtr]) < accelRate) { // if the input is effectively at the current power
            return requestedPower;
        }
        // if we need to increase speed and we are going forward
        else if (requestedPower > currentRampPower[mtr] && requestedPower > 0) { 
            currentRampPower[mtr] = currentRampPower[mtr] + accelRate;
            lastRampTime[mtr] = millis();
        }
        // if we need to decrease speed and we are going forward
        else if (requestedPower < currentRampPower[mtr] && requestedPower > 0) { 
            currentRampPower[mtr] = currentRampPower[mtr] - accelRate;
            lastRampTime[mtr] = millis();
        }
        // if we need to increase speed and we are going in reverse
        else if (requestedPower < currentRampPower[mtr] && requestedPower < 0) { 
            currentRampPower[mtr] = currentRampPower[mtr] - accelRate;
            lastRampTime[mtr] = millis();
        }
        // if we need to decrease speed and we are going in reverse
        else if (requestedPower > currentRampPower[mtr] && requestedPower < 0) { 
            currentRampPower[mtr] = currentRampPower[mtr] + accelRate;
            lastRampTime[mtr] = millis();
        }
    }

    return currentRampPower[mtr];
}


/**
 * returns the stored motor value in the class
 * @param mtr the motor number to get, an array index, so 0 -> mtr 1, etc...
 * @return returns the stored motor power for a given motor
*/
float Drive::getReqMotorPwr(uint8_t mtr) {
    return this->requestedMotorPower[mtr];
}

void Drive::setReqMotorPwr(float power, uint8_t mtr) {
    this->requestedMotorPower[mtr] = power;
}

void Drive::setLastRampPwr(float power, uint8_t mtr) {
    this->lastRampPower[mtr] = power;
}

void Drive::emergencyStop() {
    M1->writelow(), M2->writelow();
    // M1.write(0); M2.write(0);
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

    Serial.print(F("  |  Left ReqPwr: "));
    Serial.print(requestedMotorPower[0]);
    Serial.print(F("  Right ReqPwr: "));
    Serial.print(requestedMotorPower[1]);

    Serial.print(F("  |  Left: "));
    Serial.print(omega_L);
    Serial.print(F("  Right: "));
    Serial.print(omega_R);


    
    // Serial.print(F("  lastRampTime "));
    // Serial.print(lastRampTime[0]);
    // Serial.print(F("  requestedPower "));
    // Serial.print(requestedPower);
    // Serial.print(F("  current "));
    // Serial.print(currentRampPower[0]);
    // Serial.print(F("  requestedPower - currentRampPower "));
    // Serial.println(requestedPower - currentRampPower[mtr], 10);

    Serial.print(F("  Left Motor: "));
    Serial.print(requestedMotorPower[0]);
    Serial.print(F("  Right: "));
    Serial.print(requestedMotorPower[1]);

    Serial.print(F("\n"));
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
    printDebugInfo();

    // get the ramp value
    requestedMotorPower[0] = ramp(requestedMotorPower[0], 0);
    requestedMotorPower[1] = ramp(requestedMotorPower[1], 1);

    // Set the ramp value to a function, needed for generateMotionValues
    lastRampPower[0] = requestedMotorPower[0];
    lastRampPower[1] = requestedMotorPower[1];
    
    M1->write(requestedMotorPower[0]);
    M2->write(requestedMotorPower[1]);
}

