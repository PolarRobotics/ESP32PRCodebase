#pragma once

#ifndef MOTOR_TYPES_H
#define MOTOR_TYPES_H

#include <Utilities/Pair.h>

#define NUM_MOTOR_TYPES 6

// Motor types can be found here: 
// https://docs.google.com/spreadsheets/d/1DswoEAcry9L9t_4ouKL3mXFgDMey4KkjEPFXULQxMEQ/edit#gid=141541655

typedef enum {
  big_ampflow,     // index 0 for 24v AmpFlow motor E30-400-24
  small_ampflow,   // index 1 for 24v AmpFlow motor E30-150-24
  pancake_ampflow, // index 2 for 24v AmpFlow motor P40-350-24
  mecanum,         // index 3 for small 12v mecanum motors
  falcon,          // index 4 for the falcon motors on the runningback
  small_12v        // index 5 for the small 12v motors on the old robots
} MotorType;

const int MOTOR_MAX_RPM_ARR[NUM_MOTOR_TYPES] = {
  5700, // 24v AmpFlow motor E30-400-24
  5600, // 24v AmpFlow motor E30-150-24
  3500, // 24v AmpFlow motor P40-350-24
  6000, // small 12v mecanum motors
  6380, // the falcon motors on the runningback
  4000  // !TEMP, NEED TO CONFIRM NUMBER the small 12v motors on the old robots
};

const char* getMotorTypeString(MotorType type);

/**
 * @brief coeffs deffins the two constants needed for a power curve fit
 * 
 * f(x)=ax^b
 * where, a and b are constonts
 * 
 */
typedef struct Coeffs{
  float a;
  float b;
} coeffs_t;

coeffs_t getMotorCurveCoeff(MotorType motor, bool negativeDir = false);

#endif // MOTOR_TYPES_H