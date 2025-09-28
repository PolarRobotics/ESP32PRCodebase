#pragma once

#ifndef MOTOR_TYPES_H
#define MOTOR_TYPES_H

#include <Utilities/Pair.h>

#define NUM_MOTOR_TYPES 7

// Motor types can be found here: 
// https://docs.google.com/spreadsheets/d/1DswoEAcry9L9t_4ouKL3mXFgDMey4KkjEPFXULQxMEQ/edit#gid=141541655

typedef enum {
  big_ampflow,     // index 0 for 24v AmpFlow motor E30-400-24
  small_ampflow,   // index 1 for 24v AmpFlow motor E30-150-24
  pancake_ampflow, // index 2 for 24v AmpFlow motor P40-350-24
  mecanum,         // index 3 for small 12v mecanum motors
  falcon,          // index 4 for the falcon motors on the runningback
  neo_vortex,      // index 5 for the neo vortex motors on the new runningback  
  small_12v        // index 6 for the small 12v motors on the old robots
} MotorType;

const int MOTOR_MAX_RPM_ARR[NUM_MOTOR_TYPES] = {
  5700, // index 0 24v AmpFlow motor E30-400-24
  5600, // index 1 24v AmpFlow motor E30-150-24
  3500, // index 2 24v AmpFlow motor P40-350-24
  6000, // index 3 small 12v mecanum motors
  6380, // index 4 the falcon motors on the runningback
  6700, // index 5 for the neo vortex motors on the new runningback
  4000  // index 6 //!TEMP, NEED TO CONFIRM NUMBER the small 12v motors on the old robots
};

const char* getMotorTypeString(MotorType type);

#endif // MOTOR_TYPES_H