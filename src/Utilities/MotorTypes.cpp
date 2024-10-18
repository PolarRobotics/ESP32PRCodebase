#include "MotorTypes.h"

constexpr Pair<MotorType, const char*> 
motorTypeStrings[NUM_MOTOR_TYPES] = {
  { big_ampflow,      "big_ampflow"     },
  { small_ampflow,    "small_ampflow"   },
  { pancake_ampflow,  "pancake_ampflow" },
  { mecanum,          "mecanum"         },
  { falcon,           "falcon"          },
  { small_12v,        "small_12v"       }
};



/**
 * @brief getMotorTypeString returns a string correlating to the passed motor type enum
 * @example for MotorType `falcon` the function returns "falcon"
 * @param type the enum to be converted to a string
 * @return a const char* representing the motor type
 */
const char* getMotorTypeString(MotorType type) {
  return motorTypeStrings[static_cast<int>(type)].value;
}

/**
 * @brief hanedls rutuning the coffictions needed for the cureve fit of the motors
 * these cureve fits are based on real data collected expermentaly 
 * 
 * @return two constonats in the coeffs struct
 * f(x)=ax^b
 * where, a and b are constonts
 * return {a,b};
 */
coeffs_t getMotorCurveCoeff(MotorType motor, bool negativeDir){
  if(negativeDir){
    //motor is moving CW
    switch (motor) {
      case MotorType::big_ampflow: return {.0012f, .7895f};
      case MotorType::small_ampflow: return {1.0f, 1.0f};
      case MotorType::mecanum: return {1.0f, 1.0f};
      case MotorType::falcon: return {1.0f, 1.0f};
    }
  } else {
    //motor is moving CCW
    switch (motor) {
      case MotorType::big_ampflow: return {.0087f, .5616f};
      case MotorType::small_ampflow: return {1.0f, 1.0f};
      case MotorType::mecanum: return {1.0f, 1.0f};
      case MotorType::falcon: return {1.0f, 1.0f};
    }
  }
  return {1.0f, 1.0f};
}