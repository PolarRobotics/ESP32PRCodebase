#include <Arduino.h>
#include "MotorInterface.h"

/**
 * @brief 
 * A class similar to the servo class, implements a method of choosing timers and channels 
 * based on the number of motors attached, uses writeMicros, etc. Utilizes the LedC built-in functions
 * 
 * since the sabretooth is expecting a duty cycle difference (time the signal is high for) 
 * I wrote a function to convert the time on value to a duty cycle, this would need to be moved into the Control class,
 * 
 * Goals/ Design specs: 
 *  - be able to declare multiple motor objects, with pin definitions.
 *  - avoid two motors on one pin and timers overlapping
 *  - write a value to the motor, maybe -100 100, we can play around with this
 *  - needs to be usable in drive and special bot classes
 * 
 * Example uses:
 * Definition: the motor object type can simply be defined by naming the motor object using the Motor type:
 *      Motor ExampleMotorObj;
 *   
 * and the pin can be attached by calling attach():
 *      ExampleMotorObj.attach();
 * 
 * to write to the motor simply pass a float between -1 and 1 into write()
 *      ExampleMotorObj.write(power);
 * 
 * where -1, 0 and 1 correspond to the throttles in the reverse, stop and forward directions
 * 
 * @author Rhys Davies 
 */
MotorInterface::MotorInterface() {
    if(MotorCount < MAX_NUM_MOTORS)
        this->motorIndex = MotorCount++;  // assign a servo index to this instance
    else
        this->motorIndex = 255;
    // this->motorIndex = MotorCount < MAX_NUM_MOTORS ? MotorCount++ : 255;
}

/**
 * @brief attach the given pin to the next free channel, returns channel number or 255 if failure
 * @author Rhys Davies
 * Updated 2-26-2023
 * 
 * @param pin the pin the motor is connected to
 * @return uint8_t the channel number the pin is attached to, 255 if failure
 */
uint8_t MotorInterface::attach(int pin) {
    return attach(pin, MIN_PWM_US, MAX_PWM_US);
}

/**
 * @brief attach the given pin to the next free channel, returns channel number or 255 if failure
 * @author Rhys Davies
 * Updated 2-26-2023
 * 
 * @param pin the pin the motor is connected to
 * @param min min on time in us
 * @param max max on time in us
 * @return uint8_t the channel number the pin is attached to, 255 if failure
 */
uint8_t MotorInterface::attach(int pin, int min, int max) {
    if(this->motorIndex < MAX_NUM_MOTORS - 1) {
        // pinMode(pin, OUTPUT);                             // set servo pin to output
        // digitalWrite(pin, LOW);                           // set the servo pin to low to avoid spinouts
        motors[this->motorIndex].pin = pin;                  // assign this servo a pin
        // motors[this->motorIndex].isactive = true;            // set the servo to active
        motors[this->motorIndex].channel = this->motorIndex; // set the servo ledc channel
        this->min = min; 
        this->max = max;
        ledcSetup(this->motorIndex, PWM_FREQ, PWM_RES);
        ledcAttachPin(pin, this->motorIndex);
        ledcWrite(this->motorIndex, 0);
    }
    return this->motorIndex;
}

/**
 * @brief write writes the pwm duty cycle, converted from the power input to the pin
 * @author Rhys Davies
 * Updated 2-26-2023
 * 
 * input range:  [-1, 1]
 * 
 * @param pwr input power
*/
void MotorInterface::write(float pwr) {
    ledcWrite(this->motorIndex, power2Duty(pwr));
}

void MotorInterface::displayPinInfo() {
    Serial.print(F("Motor: "));
    Serial.print(this->motorIndex);
    Serial.print(F(" on Pin #"));
    Serial.print(motors[this->motorIndex].pin);
    Serial.print(F(" Channel: "));
    Serial.print(motors[this->motorIndex].channel);
    Serial.print(F("\r\n"));

    // Serial.print(F("\r\nDuty Cycle: "));
    // Serial.print(power2Duty(pwr));
}


/**
 * @brief power2Duty convert the [-1, 1] motor value to a timeon value is microseconds, 
 * then converts to a duty cycle and then scales that to a 16-bit integer, the resolution of the channel
 * link to math model: https://www.desmos.com/calculator/kmrh5pjrdf
 * @author Rhys Davies
 * Updated 2-26-2023
 * 
 * input range:  [-1, 1]
 * output range: [0, 65,536] (Really 26215 to 52429)
 * 
 * @param power the input power
*/
uint16_t MotorInterface::power2Duty(float power) {
    // this can be written in compiler code, but we are trying to save on flash memory
    this->tempTimeon = (power + 1) * 500 + 1000;
    return (tempTimeon / (PWM_PERIOD * 1000)) * (PWM_MAXDUTY / 1000);
}

/**
 * @brief writelow hopefully helps to prevent the issue with motors spinning at 
 * max speed on boot, sending bots into the stratosphere
 * 
 */
void MotorInterface::writelow() {
    // digitalWrite(motors[this->motorIndex].pin, LOW);
    write(0);
}