#include "MotorTypes.h"

constexpr Pair<eMOTOR_TYPE, const char*> 
motorTypeStrings[NUM_MOTOR_TYPES] = {
  {  big,     "big"      },
  {  small,   "small"    },
  {  mecanum, "mecanum"  },
  {  falcon,  "falcon"   }
};

/**
 * @brief getMotorTypeString returns a string correlating to the passed motor type enum
 * @example for eMOTOR_TYPE `falcon` the function returns "falcon"
 * @param type the enum to be converted to a string
 * @return a const char* representing the motor type
 */
const char* getMotorTypeString(eMOTOR_TYPE type) {
  return motorTypeStrings[static_cast<int>(type)].value;
}