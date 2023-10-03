#ifndef MOTOR_TYPES_H
#define MOTOR_TYPES_H

#include <Utilities/Pair.h>

#define NUM_MOTOR_TYPES 6

// Motor types can be found here: 
// https://docs.google.com/spreadsheets/d/1DswoEAcry9L9t_4ouKL3mXFgDMey4KkjEPFXULQxMEQ/edit#gid=141541655

typedef enum {
  big_ampflow,     // MOTOR_TYPE value of 0 for 24v AmpFlow motor E30-400-24
  small_ampflow,   // MOTOR_TYPE value of 1 for 24v AmpFlow motor E30-150-24
  pancake_ampflow, // MOTOR_TYPE value of 2 for 24v AmpFlow motor P40-350-24
  mecanum,         // MOTOR_TYPE value of 3 for small 12v mecanum motors
  falcon,          // MOTOR_TYPE value of 4 for the falcon motors on the runningback
  small_12v        // MOTOR_TYPE value of 5 for the small 12v motors on the old robots
} MotorType;

const char* getMotorTypeString(MotorType type);

#endif // MOTOR_TYPES_H