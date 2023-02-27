
#include <Arduino.h>

#include "MotorControl.h"

MotorControl::MotorControl() {
    if( ServoCount < MAX_NUM_MOTORS)
        this->servoIndex = ServoCount++;                    // assign a servo index to this instance
    else
        this->servoIndex = 255;
}

uint8_t MotorControl::attach(int pin) {
    return attach(pin, MIN_PWM_US, MAX_PWM_US);
}
/**
 * @brief attach the given pin to the next free channel, returns channel number or 255 if failure
 * @author Rhys Davies
 * Updated 2-26-2023
*/
uint8_t MotorControl::attach(int pin, int min, int max) {
    if(this->servoIndex < MAX_NUM_MOTORS - 1) {
        // pinMode( pin, OUTPUT);                            // set servo pin to output
        servos[this->servoIndex].pin = pin;                  // assign this servo a pin
        servos[this->servoIndex].isactive = true;            // set the servo to active
        servos[this->servoIndex].channel = this->servoIndex; // set the servo channel
        this->min = min; 
        this->max = max;
        ledcAttachPin(pin, this->servoIndex);
        ledcSetup(this->servoIndex, PWM_FREQ, PWM_RES);
    }
    return this->servoIndex;
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

    ledcWrite(this->servoIndex, power2Duty(pwr));
}

/**
 * @brief power2Duty convert the [-1, 1] motor value to a timeon value is microseconds, 
 * then converts to a duty cycle and then scales that to a 16-bit #, the resolution of the channel
 * @author Rhys Davies
 * Updated 2-26-2023
 * 
 * input range:  [-1, 1]
 * output range: [0, 65,536]
 * 
 * @param power the input power
*/
uint16_t MotorControl::power2Duty(float power) {
    // this can be written in compiler code, but we are trying to save on flash memory
    uint32_t tempTimeon = (power + 1) * 500 + 1000;
    tempTimeon = constrain(tempTimeon, this->min, this->max);
    return (tempTimeon / PWM_PERIOD_US) * (pow(2, PWM_RES) / 1000);
}
