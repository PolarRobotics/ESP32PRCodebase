#pragma once

#define NUM_MOTOR_TYPES 4

typedef enum {
  big,      // MOTOR_TYPE value of 0 for 24v AmpFlow motor E30-400
  small,    // MOTOR_TYPE value of 1 for 24v AmpFlow motor E30-150
  mecanum,  // MOTOR_TYPE value of 2 for small 12v mecanum motors
  falcon    // MOTOR_TYPE value of 3 for the falcon motors on the runningback
} eMOTOR_TYPE;

constexpr Pair<eMOTOR_TYPE, const char*> motorTypeNames[NUM_MOTOR_TYPES] = {
  {  big,     "big"      },
  {  small,   "small"    },
  {  mecanum, "mecanum"  },
  {  falcon,  "falcon"   }
};

/**
 * @brief getMotorTypeName returns a string correlating to the passed motor type enum
 * @example for eMOTOR_TYPE `falcon` the function returns "falcon"
 * @param type the enum to be converted to a string
 * @return a const char* representing the motor type
 */
const char* getMotorTypeName(eMOTOR_TYPE type) {
  return motorTypeNames[static_cast<int>(type)].value;
}