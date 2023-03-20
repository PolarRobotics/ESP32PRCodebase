#pragma once

#ifndef __MOTOR_CONTROL__
#define __MOTOR_CONTROL__

#include <Arduino.h>

#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

#define MAX_NUM_MOTORS 6

#define MAX_PWM_US 2000
#define MIN_PWM_US 1000

//PWM defines:
#define PWM_RES 16 //channel resolution in bits, this is a really high resolution, can lower this if we have stability problems
// #define PWM_MAXDUTY 65535    // (2^16) - 1
const int PWM_MAXDUTY = (1 << PWM_RES) - 1;
#define USE_TIMER MCPWM_TIMER_0 //desegnated timer for pwm signals

// a Period of 2500us for the sabertooth, gives the st enough time to react to inputs, 
// can make this value closer to 2000us if we have issues with the ST not updating fast enough
#define PWM_PERIOD 0.0025   // 2500 us
#define PWM_FREQ 1/PWM_PERIOD
// #define PWM_FREQ 1000000 / PWM_PERIOD_US

// macro to clean up the screen
#define SERVOS servos[this->motorIndex]

// class MotorControl;

typedef struct servo {
    uint8_t pin;
    // bool isactive;
    mcpwm_unit_t mcunitnum;
    mcpwm_io_signals_t mcoutputmodule; // which module to use for the pwm signal
    uint8_t channel;
} servo_t;

static servo_t servos[MAX_NUM_MOTORS];
static uint8_t ServoCount = 0;

static const mcpwm_unit_t MCUnitNumREF[MCPWM_UNIT_MAX] = {
    MCPWM_UNIT_0, MCPWM_UNIT_1
};

static const mcpwm_io_signals_t MCOutputREF[MAX_NUM_MOTORS] = {
    MCPWM0A,        /*!<PWM0A output pin*/
    MCPWM0B,        /*!<PWM0B output pin*/
    MCPWM1A,        /*!<PWM1A output pin*/
    MCPWM1B,        /*!<PWM1B output pin*/
    MCPWM2A,        /*!<PWM2A output pin*/
    MCPWM2B,        /*!<PWM2B output pin*/
};

class MotorControl {
private:
    mcpwm_config_t pwm_config0;
    uint8_t motorIndex;  // index into the channel data for this servo
    int8_t min;          // minimum is this value times 4 added to MIN_PULSE_WIDTH    
    int8_t max;          // maximum is this value times 4 added to MAX_PULSE_WIDTH   
    uint32_t tempTimeon;
    uint16_t power2Duty(float power);
public:
    MotorControl();
    uint8_t attach(int pin);           
    uint8_t attach(int pin, int min, int max); // as above but also sets min and max values for writes. 
    void write(float pwr);
    void displayPinInfo();
    void writelow();
};

#endif // !__MOTOR_CONTROL__