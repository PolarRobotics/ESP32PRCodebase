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

Drive::Drive(BotType botType, MotorType motorType) {
  this->botType = botType;
  this->motorType = motorType;

  switch (botType) {
    case receiver:
      this->turnMax = 0.8;
      this->turnMin = 0.8;
      break;
    case center:
      this->turnMax = 0.2;
      this->turnMin = 0.2;
      break;
    case quarterback:
      this->turnMax = 0.4;
      this->turnMin = 0.4;
      break;
    case runningback:
      this->turnMax = 0.5;
      this->turnMin = 0.2;
      break;
    default:
      this->turnMax = 0.65;
      this->turnMin = 0.65;
  }

  if (botType == quarterback) {
    this->BIG_BOOST_PCT = 0.8; 
    this->BIG_NORMAL_PCT = 0.4; 
    this->BIG_SLOW_PCT = 0.3;
  } else {
    this->BIG_BOOST_PCT = 1.0;  
    this->BIG_NORMAL_PCT = 0.8; 
    this->BIG_SLOW_PCT = 0.4;
  }

  // initialize arrays
  for (int i = 0; i < NUM_MOTORS; i++) {
    requestedMotorPower[i] = 0.0f;
    currentRampPower[i] = 0.0f;
    lastRampPower[i] = 0.0f;
    turnMotorValues[i] = 0.0f;
  }
}

void Drive::setServos(uint8_t lpin, uint8_t rpin) {
    //this->motorPins[0] = lpin, this->motorPins[1] = rpin;
    M1.attach(lpin), M2.attach(rpin);
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
    if (fabs(stickForwardRev) < STICK_DEADZONE) {
      stickForwardRev = 0;
    }
    if (fabs(stickTurn) < STICK_DEADZONE) {
      stickTurn = 0;
    }
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
            if(stickTurn > STICK_DEADZONE) { // turn Right
                switch(abs((BSNscalar * stickForwardRev)) > abs(lastRampPower[0])) {
                    case true: calcTurningMotorValues(stickTurn, abs(lastRampPower[0]), 1); break;
                    case false: calcTurningMotorValues(stickTurn, abs(BSNscalar * stickForwardRev), 1); break;
                }
                //calcTurningMotorValues(stickTurn, lastRampPower[0], 1);
                requestedMotorPower[0] = copysign(turnMotorValues[0], stickForwardRev);
                requestedMotorPower[1] = copysign(turnMotorValues[1], stickForwardRev);
            } else if(stickTurn < -STICK_DEADZONE) { // turn Left
                switch(abs((BSNscalar * stickForwardRev)) > abs(lastRampPower[1])) {
                    case true: calcTurningMotorValues(stickTurn, abs(lastRampPower[1]), 0); break;
                    case false: calcTurningMotorValues(stickTurn, abs(BSNscalar * stickForwardRev), 0); break;
                }
                //calcTurningMotorValues(stickTurn, lastRampPower[1], 0);
                requestedMotorPower[0] = copysign(turnMotorValues[1], stickForwardRev);
                requestedMotorPower[1] = copysign(turnMotorValues[0], stickForwardRev);
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
    
    
    //R_Min = R_Min + abs(stickForwardRev)*(R_High_Min - R_Min); // start of turn scaling 

    R = (1-abs(stickTrn))*(R_Max-R_Min) + R_Min;
    
    Omega_r = abs(stickForwardRev)*BSNscalar*max_RPM;
    
    Omega_rL = (Omega_r/R)*(R+(wheelBase/2));
    Omega_rR = (Omega_r/R)*(R-(wheelBase/2));

    if ((Omega_rL > max_RPM) || (Omega_rR > max_RPM)){
        Omega_rL = max_RPM;
        Omega_rR = (max_RPM/R)*(R-(wheelBase/2));
    }

    turnMotorValues[0] = Omega_rL/max_RPM;
    turnMotorValues[1] = Omega_rR/max_RPM;

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
 * @brief Limit motor values to the -1.0 to +1.0 range.
 * Stolen wholesale from WPILib.
 */
float Drive::applyClamp(float value) {
  if (value > 1.0f) {
    return 1.0f;
  } else if (value < -1.0f) {
    return -1.0f;
  }
  return value;
}

/**
 * @brief Returns 0.0 if the given value is within the specified deadzone/range around zero. 
 * The remaining range from the deadzone to +/-1.0 is scaled from 0.0 to +/-1.0.
 * 
 * Also (mostly) stolen wholesale from WPILib.
 *
 * @param value    value to adjust
 * @param deadzone range around zero, represents [-deadzone, +deadzone]
 */
float Drive::applyDeadzone(float value, float deadzone) {
  if (value > deadzone) { // value is positive
    return (value - deadzone) / (1.0f - deadzone);
  } else if (value < -deadzone) { // value is negative
    return (value + deadzone) / (1.0f - deadzone);
  } else { // value is within deadzone
    return 0.0f;
  }
}

/**
 * @brief constant curvature hybrid drive based on WPILib: https://github.com/wpilibsuite/allwpilib/blob/96e9a6989ce1688f3edb2d9b9d21ef8cd3861579/wpilibc/src/main/native/cpp/Drive/DifferentialDrive.cpp#L117
 * 
 * Uses arcade-style constant curvature model, with override for
 * in-place (tank) turning if stickForwardRev is in deadzone (is zero).
 * 
 * Replaces generateMotionValues() and by extension calcTurningMotorValues().
 *
 * @author Max Phillips
 *
 * @param stickForwardRev left stick Y, [-1, +1], forward is positive, reverse is negative.
 * 
 * @param stickTurn right stick X, [-1, +1], left is negative, right is positive.
 * 
 * Outputs (directly modified):
 *   requestedMotorPower[0]
 *   requestedMotorPower[1]
 * 
 */
void Drive::diffDriveCurve(float stickForwardRev, float stickTurn) {
  stickForwardRev = applyClamp(stickForwardRev);

  stickTurn = applyClamp(stickTurn);

  // "zero" is defined as "within [-deadzone, +deadzone]"
  // this also helps dodge floating-point comparison problems

  if (fabs(stickForwardRev) < STICK_DEADZONE && fabs(stickTurn) < STICK_DEADZONE) { 
    // fwd stick is zero
    
    // if () { 
      // turn stick is zero
      // both sticks are zero == no movement desired, so set motors to zero
      requestedMotorPower[0] = 0, 
      requestedMotorPower[1] = 0; 
    // } else if (stickTurn > STICK_DEADZONE) { 
    //   // turn stick is positive: turn right *in-place*.
    //   requestedMotorPower[0] =  BSNscalar * abs(stickTurn);
    //   requestedMotorPower[1] = -BSNscalar * abs(stickTurn);
    // } else if (stickTurn < -STICK_DEADZONE) { 
    //   // turn stick is negative: turn left *in-place*.
    //   requestedMotorPower[0] = -BSNscalar * abs(stickTurn);
    //   requestedMotorPower[1] =  BSNscalar * abs(stickTurn);
    // }

  } else { 
    // fwd stick is not zero

    if (fabs(stickForwardRev) > STICK_DEADZONE && fabs(stickTurn) < STICK_DEADZONE) {
      // turn stick is zero, just move forward
      requestedMotorPower[0] = BSNscalar * stickForwardRev;
      requestedMotorPower[1] = BSNscalar * stickForwardRev;
    } else { 
      // moving forward and turning

      angularPower = fabs(stickForwardRev) * stickTurn * CURVE_DRIVE_TURN_SCALAR;

      // calculate initial powers. these may be above 1. this is ok for now.
      intermediateMotorPower[0] = stickForwardRev + angularPower;
      intermediateMotorPower[1] = stickForwardRev - angularPower;
      
      
      if ((fabs(stickTurn) * TURN_OVERPOWER_THRESHOLD) > fabs(stickForwardRev)) {
        // turn power overpowers forward power (configurable with TURN_OVERPOWER_THRESHOLD)
        turnOverpowerMagnitude = ((stickTurn * CURVE_DRIVE_TURN_SCALAR) - (angularPower / TURN_OVERPOWER_THRESHOLD));
        intermediateMotorPower[0] +=  copysign(turnOverpowerMagnitude, stickTurn); // take sign of turn stick
        intermediateMotorPower[1] += -copysign(turnOverpowerMagnitude, stickTurn); // above, but negate for right motor
      }

      // normalize powers
      maxMagnitude = max(fabs(intermediateMotorPower[0]), fabs(intermediateMotorPower[1]));

      if (maxMagnitude > 1.0) {
        intermediateMotorPower[0] /= maxMagnitude;
        intermediateMotorPower[1] /= maxMagnitude;
      }

      // set final motor powers
      requestedMotorPower[0] = intermediateMotorPower[0] * BSNscalar;
      requestedMotorPower[1] = intermediateMotorPower[1] * BSNscalar;

    }
  }
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
    M1.writelow(), M2.writelow();
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

    // Serial.print(F("  |  Turn: "));
    // Serial.print(lastTurnPwr);

    Serial.print(F("  |  AP: "));
    Serial.print(angularPower);

    Serial.print(F("  |  tOPM: "));
    Serial.print(turnOverpowerMagnitude);

    Serial.print(F("  |  L iPwr: "));
    Serial.print(intermediateMotorPower[0]);
    Serial.print(F("  R iPwr: "));
    Serial.print(intermediateMotorPower[1]);

    Serial.print(F("  |  Left ReqPwr: "));
    Serial.print(requestedMotorPower[0]);
    Serial.print(F("  Right ReqPwr: "));
    Serial.print(requestedMotorPower[1]);



    // Serial.print(F("  |  L: "));
    // Serial.print(Omega_rL);
    // Serial.print(F("  R: "));
    // Serial.print(Omega_rR);


    
    // Serial.print(F("  lastRampTime "));
    // Serial.print(lastRampTime[0]);
    // Serial.print(F("  requestedPower "));
    // Serial.print(requestedPower);
    // Serial.print(F("  current "));
    // Serial.print(currentRampPower[0]);
    // Serial.print(F("  requestedPower - currentRampPower "));
    // Serial.println(requestedPower - currentRampPower[mtr], 10);

    // Serial.print(F("  L Final: "));
    // Serial.print(requestedMotorPower[0]);
    // Serial.print(F("  R Final: "));
    // Serial.print(requestedMotorPower[1]);

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
    // generateMotionValues();
    diffDriveCurve(stickForwardRev, stickTurn);
    printDebugInfo();

    // get the ramp value
    requestedMotorPower[0] = ramp(requestedMotorPower[0], 0, ACCEL_RATE_TEST);
    requestedMotorPower[1] = ramp(requestedMotorPower[1], 1, ACCEL_RATE_TEST);

    // Set the ramp value to a function, needed for generateMotionValues
    lastRampPower[0] = requestedMotorPower[0];
    lastRampPower[1] = requestedMotorPower[1];
    
    M1.write(requestedMotorPower[0]);
    M2.write(requestedMotorPower[1]);
}

