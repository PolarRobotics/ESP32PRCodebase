#include <Arduino.h>
#include "MotorControl.h"

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
MotorControl::MotorControl() {
    if(ServoCount < MAX_NUM_MOTORS)
        this->motorIndex = ServoCount++;  // assign a servo index to this instance
    else
        this->motorIndex = 255;
    // this->motorIndex = ServoCount < MAX_NUM_MOTORS ? ServoCount++ : 255;
}

/**
 * @brief attach the given pin to the next free channel, returns channel number or 255 if failure
 * @author Rhys Davies
 * Updated 2-26-2023
 * 
 * @param pin the pin the motor is connected to
 * @return uint8_t the channel number the pin is attached to, 255 if failure
 */
uint8_t MotorControl::attach(int pin) {
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
uint8_t MotorControl::attach(int pin, int min, int max) {
    if(this->motorIndex < MAX_NUM_MOTORS - 1) {
        // pinMode(pin, OUTPUT);                             // set servo pin to output
        // digitalWrite(pin, LOW);                           // set the servo pin to low to avoid spinouts
        servos[this->motorIndex].pin = pin;                  // assign this servo a pin
        // servos[this->motorIndex].isactive = true;            // set the servo to active
        servos[this->motorIndex].channel = this->motorIndex; // set the servo ledc channel
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
void MotorControl::write(float pwr) {
    ledcWrite(this->motorIndex, power2Duty(pwr));
}

void MotorControl::displayPinInfo() {
    Serial.print(F("Motor: "));
    Serial.print(this->motorIndex);
    Serial.print(F(" on Pin #"));
    Serial.print(servos[this->motorIndex].pin);
    Serial.print(F(" Channel: "));
    Serial.print(servos[this->motorIndex].channel);
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
uint16_t MotorControl::power2Duty(float power) {
    // this can be written in compiler code, but we are trying to save on flash memory
    this->tempTimeon = (power + 1) * 500 + 1000;
    return (tempTimeon / (PWM_PERIOD * 1000)) * (PWM_MAXDUTY / 1000);
}

/**
 * @brief writelow hopefully helps to prevent the issue with motors spinning at 
 * max speed on boot, sending bots into the stratosphere
 * 
 */
void MotorControl::writelow() {
    digitalWrite(servos[this->motorIndex].pin, LOW);
}


// Old code:

// /**
//  * alternate for servos writeMicroseconds, a function to set the motors based on a power input (-1 to 1),
//  * manually sets the motor high for the pwm time.
//  * @author Rhys Davies
//  * Created: 10-5-2022
//  * Updated: 10-29-2022
//  *
//  * Set the motors by outputting a pulse with a period corresponding to the motor power,
//  * determined by Convert2PWMVal. These calculations can not be put into a function,
//  * because it confuses the compiler as these are time critical tasks.
//  * 
//  * FUTURE: write this so both pins are HIGH at the same time, and the one that goes low first is called,
//  * because the beginning of the pulse is at the same time for both pins, but this isnt entirely necessary
//  * considering both motors operate independently on the sabertooth
//  *
//  * @param pwr the motor power to be set
//  * @param pin the motor to be set (0 for left, 1 for right)
// */
// void Drive::setMotorPWM(float pwr, byte pin) {
//     digitalWrite(motorPins[pin], HIGH);
//     delayMicroseconds(Convert2PWM(pwr) - 40);
//     digitalWrite(motorPins[pin], LOW);
//     delayMicroseconds(2000 - Convert2PWM(pwr) - 40); //-170
//     // digitalWrite(motorPins[0], HIGH);
//     // delayMicroseconds(Convert2PWMVal(motorPower[0]) - 40);
//     // digitalWrite(motorPins[0], LOW);
//     // // delayMicroseconds(2000 - Convert2PWMVal(motorPower[0]) - 40); //-170
//     // digitalWrite(motorPins[1], HIGH);
//     // delayMicroseconds(Convert2PWMVal(motorPower[1]) - 40);
//     // digitalWrite(motorPins[1], LOW);
//     // delayMicroseconds(2000 - Convert2PWMVal(motorPower[1]) - 40); //-170
// }
