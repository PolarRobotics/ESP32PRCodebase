#pragma once

#ifndef __PWM_MODULE__
#define __PWM_MODULE__

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

#define MAX_NUM_MOTORS 16

// input of 0 is 1500 us
#define MAX_PWM_US 2000     // Input of 1
#define MIN_PWM_US 1000     // Input of -1

#define PWM_ADDRESS 0x40    // Default PWM i2c address
#define PWM_RES 12          // Max PWM Resolution is 12 https://cdn-shop.adafruit.com/datasheets/PCA9685.pdf
#define PWM_MAXDUTY (1 << PWM_RES) - 1
// a Period of 2500us for the sabertooth, gives the st enough time to react to inputs, 
// can make this value closer to 2000us if we have issues with the ST not updating fast enough
#define PWM_PERIOD 0.002   // 2500 us
#define PWM_FREQ 1/0.0022   // 

typedef struct servo{
  uint8_t pin;
}servo_t;

static servo_t motors[MAX_NUM_MOTORS];
static uint8_t MotorCount = 0;

class PWM_Module{
private:
    uint8_t motorIndex;     // Index of the Motor
    int8_t min;             
    int8_t max;             // maximum PWM value, set based on motor driver (sabertooth is MIN_PWM_US)
    uint32_t tempTimeon;
    uint16_t power2Duty(float power);
    static Adafruit_PWMServoDriver* pwm_test_module; // Singleton instance
public:
    PWM_Module();
    uint8_t attach(int pin, int min, int max);
    void write(float power);
    void displayPinInfo();
    void writelow();
    static Adafruit_PWMServoDriver* getPWMInstance(); // Method to get the instance
};

// extern Adafruit_PWMServoDriver PWM_Module::pwm_test_module; // Declare as extern

#endif