#include <Arduino.h>
#include <Drive/Drive.h>
#include "DriveQuick.h"

/*
drive quick is meant for the experimental running back
Features:
    slower ramp time
    sharper turning?
        should have tank mode turning at lower speeds
        and standard turning at higher speeds
    normal percent speed: 0.6
    2x normal 12v motors?
    1x omniwheel
*/

//! Must call base class constructor with appropriate arguments
DriveQuick::DriveQuick(drive_param_t driveParams) : Drive(BotType::runningback, MotorType::falcon, driveParams, false, 0) {
  // initialize array
  // for (int i = 0; i < NUM_MOTORS; i++) {
  //   falcon_motor_pwr[i] = 0.0f;
  // }
}

/**
 * @brief 
 * prototype turning model: https://www.desmos.com/calculator/pjyj3tjwym
 * 
 * some things to note: for this turning model, going full forward, r = 1, both motors get set to 0.707 (sqrt(2))
 * and when you are tank turning left or right, that motor gets set to 1, so turning would be faster than
 * going forward, this math model may need some tweaking, but this is something we can test for the future 
 *  
 */
// void DriveQuick::generateMotionValues() {
//     // generate the motion vector in polar form
//     this->r = hypot(getForwardPower(), getTurnPower());
//     this->falconTurnPwr = atan2(getForwardPower(), getTurnPower());

//     // ensure the magnitude of the speed does not go over 1 and multiply it by the bsn value
//     this->r = constrain(this->r, 0, 1) * getBSN(); 
    
//     // set both motor powers
//     falcon_motor_pwr[0] = r * sin(this->falconTurnPwr + (PI/4)); // calculate turning for left wheel
//     falcon_motor_pwr[1] = r * cos(this->falconTurnPwr + (PI/4)); // calculate turning for right wheel
// }

/**
 * @brief updates the motors after calling all the functions to generate
 * turning and scaling motor values, the intention of this is so the
 * programmer doesnt have to call all the functions, this just handles it,
 * reducing clutter in the main file.
 * DO NOT CALL THIS FUNCTION UNTIL setStickPwr and setBSN have been called
 * @author Rhys Davies
 * Created: 9-12-2022
 * Updated: 10-11-2020
*/
void DriveQuick::update() {
  // Generate turning motion
  generateMotionValues(RB_TANK_MODE_PCT);

  // calculate the ramped power
  // falcon_motor_pwr[0] = ramp(falcon_motor_pwr[0], 0);
  // falcon_motor_pwr[1] = ramp(falcon_motor_pwr[1], 1);
  // falcon_motor_pwr[0] = M1.ramp(getReqMotorPwr(0), RB_ACCELERATION_RATE);
  // falcon_motor_pwr[1] = M2.ramp(getReqMotorPwr(1), RB_ACCELERATION_RATE);

  requestedMotorPower[0] = M1.ramp(requestedMotorPower[0], RB_ACCELERATION_RATE);
  requestedMotorPower[1] = M2.ramp(requestedMotorPower[1], RB_ACCELERATION_RATE);

  // set the last ramp power, used in ramp
  // setLastRampPwr(falcon_motor_pwr[0], 0);
  // setLastRampPwr(falcon_motor_pwr[1], 1);

  // Set the ramp value to a function, needed for generateMotionValues
  lastRampPower[0] = requestedMotorPower[0];
  lastRampPower[1] = requestedMotorPower[1];

  // if the requested motor power is really small, set the motors to zero to prevent stalls
  // falcon_motor_pwr[0] = fabs(falcon_motor_pwr[0]) < MOTOR_ZERO_OFFST ? 0 : falcon_motor_pwr[0];
  // falcon_motor_pwr[1] = fabs(falcon_motor_pwr[1]) < MOTOR_ZERO_OFFST ? 0 : falcon_motor_pwr[1];

  requestedMotorPower[0] = fabs(requestedMotorPower[0]) < MOTOR_ZERO_OFFST ? 0 : requestedMotorPower[0];
  requestedMotorPower[1] = fabs(requestedMotorPower[1]) < MOTOR_ZERO_OFFST ? 0 : requestedMotorPower[1];
  
  // write calculated powers to the motors 
  // note: adding a negative, because we cant change the motor direction in hardware
  M1.write(requestedMotorPower[0]);
  M2.write(-requestedMotorPower[1]);
}

void DriveQuick::printDebugInfo() {
  Serial.print(F("DQ | "));
  Serial.print(F("L_Hat_Y: "));
  Serial.print(stickForwardRev);
  Serial.print(F("  R_HAT_X: "));
  Serial.print(stickTurn);

  // Serial.print(F("  |  Turn: "));
  // Serial.print(lastTurnPwr);

  Serial.print(F("  L_MotPwr: "));
  // Serial.print(falcon_motor_pwr[0]);
  Serial.print(requestedMotorPower[0]);

  Serial.print(F("  R_MotPwr: "));
  // Serial.print(falcon_motor_pwr[1]);
  Serial.print(requestedMotorPower[1]);

  Serial.print(F("\n"));
}
