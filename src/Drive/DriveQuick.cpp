#include <Arduino.h>
#include <Drive/Drive.h>
#include <MotorControl.h>
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

/**
 * @brief 
 * prototype turning model: https://www.desmos.com/calculator/pjyj3tjwym
 * 
 * some things to note: for this turning model, going full forward, r = 1, both motors get set to 0.707 (sqrt(2))
 * and when you are tank turning left or right, that motor gets set to 1, so turning would be faster than
 * going forward, this math model may need some tweaking, but this is something we can test for the future 
 *  
 */
void DriveQuick::generateMotionValues() {
    // generate the motion vector in polar form
    this->r = hypot(getFwdRev(), getTurn());
    this->falconTurnPwr = atan2(getFwdRev(), getTurn());

    // ensure the magnitude of the speed does not go over 1 and multiply it by the bsn value
    this->r = constrain(this->r, 0, 1) * getBSN(); 
    
    // set both motor powers
    falcon_motor_pwr[0] = r * cos(this->falconTurnPwr + (PI/4)); // calculate turning for left wheel
    falcon_motor_pwr[1] = r * sin(this->falconTurnPwr + (PI/4)); // calculate turning for right wheel
}

// /**
//  * @brief needed to calibrate the falcon motors, we need to send a 100% pwm signal to 
//  * be able to calibrate the max speed
//  * 
//  */
// void DriveQuick::setMaxPWR() {

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
    generateMotionValues();

    // calculate the ramped power
    // falcon_motor_pwr[0] = ramp(falcon_motor_pwr[0], 0);
    // falcon_motor_pwr[1] = ramp(falcon_motor_pwr[1], 1);
    // falcon_motor_pwr[0] = ramp(getMotorPwr(0), 0, RB_ACCELERATION_RATE);
    // falcon_motor_pwr[1] = ramp(getMotorPwr(1), 1, RB_ACCELERATION_RATE);
    falcon_motor_pwr[0] = ramp(falcon_motor_pwr[0], 0, RB_ACCELERATION_RATE);
    falcon_motor_pwr[1] = ramp(falcon_motor_pwr[1], 1, RB_ACCELERATION_RATE);

    // set the last ramp power, used in ramp
    setLastRampPwr(falcon_motor_pwr[0], 0);
    setLastRampPwr(falcon_motor_pwr[1], 1);

    // if the requested motor power is really small, set the motors to zero to prevent stalls
    falcon_motor_pwr[0] = fabs(falcon_motor_pwr[0]) < MOTOR_ZERO_OFFST ? 0 : falcon_motor_pwr[0];
    falcon_motor_pwr[1] = fabs(falcon_motor_pwr[1]) < MOTOR_ZERO_OFFST ? 0 : falcon_motor_pwr[1];
    
    // write calculated powers to the motors 
    // note: adding a negative, because we cant change the motor direction in hardware
    M1.write(falcon_motor_pwr[0]);
    M2.write(-falcon_motor_pwr[1]); 
}

/**
 * prints the internal variables to the serial monitor in a clean format,
 * this function exists out of pure laziness to not have to comment out all the print statments
 * @author
 * Updated:
*/
void DriveQuick::printDebugInfo() {
    Serial.print(F("L_Hat_Y: "));
    Serial.print(stickForwardRev);
    Serial.print(F("  R_HAT_X: "));
    Serial.print(stickTurn);

    // Serial.print(F("  |  Turn: "));
    // Serial.print(lastTurnPwr);

    Serial.print(F("  L_MotPwr: "));
    Serial.print(falcon_motor_pwr[0]);
    Serial.print(F("  R_MotPwr: "));
    Serial.print(falcon_motor_pwr[1]);
    Serial.print(F("\n"));
}

