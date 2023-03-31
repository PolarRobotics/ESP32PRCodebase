#include "Drive/Drive.h"
#include "Drive/DriveQuick.h"

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
 * @brief updates the motors after calling all the functions to generate
 * turning and scaling motor values, the intention of this is so the
 * programmer doesnt have to call all the functions, this just handles it,
 * reducing clutter in the main file.
 * DO NOT CALL THIS FUNCTION UNTIL setStickPwr and setBSN have been called before update
 * @author Rhys Davies
 * Created: 9-12-2022
 * Updated: 10-11-2020
*/
void DriveQuick::update() {
    // Generate turning motion
    generateMotionValues();

    // get the ramp value
    // motorPower[0] = ramp(motorPower[0], 0);
    // motorPower[1] = ramp(motorPower[1], 1);

    setMotorPwr(ramp(getMotorPwr(0),0),0);
    setMotorPwr(ramp(getMotorPwr(1),1),1);

    // Set the ramp value to a function, needed for generateMotionValues
    // lastRampPower[0] = motorPower[0];
    // lastRampPower[1] = motorPower[1];

    setLastRampPwr(getMotorPwr(0),0);
    setLastRampPwr(getMotorPwr(1),1);
    
    M1.write(getMotorPwr(0));
    M2.write(-getMotorPwr(1));
}

