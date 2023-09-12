#pragma once

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

#define MAX_NUM_MOTORS 16

#define MAX_PWM_US 2000
#define MIN_PWM_US 1000

//PWM defines:
#define PWM_RES 16 //channel resolution in bits, this is a really high resolution, can lower this if we have stability problems
// #define PWM_MAXDUTY 65535    // (2^16) - 1
const int PWM_MAXDUTY = (1 << PWM_RES) - 1;
// a Period of 2500us for the sabertooth, gives the st enough time to react to inputs, 
// can make this value closer to 2000us if we have issues with the ST not updating fast enough
#define PWM_PERIOD 0.0025   // 2500 us
#define PWM_FREQ 1/PWM_PERIOD
// #define PWM_FREQ 1000000 / PWM_PERIOD_US

// class MotorControl;

// Enum for Increasing or Decreasing Flywheel Speed
enum SpeedStatus {
  INCREASE, DECREASE
};

typedef struct servo {
    uint8_t pin;
    // bool isactive;
    uint8_t channel;
} servo_t;

static servo_t servos[MAX_NUM_MOTORS];
static uint8_t ServoCount = 0;

class MotorControl {
  private:
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
    void readEncoder();
    void printSpeed();
    int calcSpeed(int current_count);
    int addInt(int x, int y);

    // for use in void readEncoder()
    int a_channel = 35;
    int b_channel = 34;
    int encoderACount = 0;
    int b_channel_state = 0;
    int rollerover = 2048;
  
    // For use in int calcSpeed()
    int prev_current_count = 0;
    int rolleroverthreshold = 500; //this is bases on the fastes speed we expect, if the differace is going to be grater a rollover has likely accured
    unsigned long current_time = 0;
    unsigned long prev_current_time = 0; 
    float omega = 0;
};

#endif // MOTOR_CONTROL_H