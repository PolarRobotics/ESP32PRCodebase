#pragma once

#define NUM_MOTOR_TYPES 4

typedef enum {
  big,          // MOTOR_TYPE value of 0 for 24v AmpFlow motor E30-400
  small,        // MOTOR_TYPE value of 1 for 24v AmpFlow motor E30-150
  mecanum_motor, // MOTOR_TYPE value of 2 for small 12v mecanum motors
  falcon_motor   // MOTOR_TYPE value of 3 for the falcon motors on the runningback
} eMOTOR_TYPE;