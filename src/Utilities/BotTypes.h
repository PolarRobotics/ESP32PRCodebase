#pragma once

#ifndef BOT_TYPES_H
#define BOT_TYPES_H

#include <Arduino.h>
#include <Utilities/Pair.h>
#include <Utilities/MotorTypes.h>

#define NUM_POSITIONS 7 // number of members of eBOT_TYPE

/** BotType
 * enum for the possible positions a robot can have on the field
 * NOTE: when this list is updated, make sure to update the botTypeStrings array with a corresponding string
 * 
 *  Robot Type Enum
 *  0: Lineman
 *  1: Receiver
 *  2: Runningback
 *  3: Center
 *  4: Mecanum Center
 *  5: Quarterback
 *  6: Kicker
*/
typedef enum {
  lineman,
  receiver,
  runningback,
  center,
  mecanum_center,
  quarterback,
  kicker
} BotType;

/**
 * @brief BotConfig robot configuration datastructure, 
 * used to read and write bot information to the esp 
 * Order to write to eeprom:
 * Bot Name index
 * Bot Type enum
 * Motor Type enum
 * Gear Ratio index
 * ? code last uploaded date and time
 */
typedef struct BotConfig {
  uint8_t index;
  const char * bot_name;
  BotType bot_type; // primary robot position
  MotorType mot_type;
  // placeholder: gear ratio index
  // uint8_t GR_index;
  // LED STRIP:
  // bool has_leds;
  // uint8_t num_leds;
  // placeholder: fHasMultipleBotTypes (for new linemen/receivers)
  // BotType secondary_type;
} bot_config_t;

#define NUM_BOTS 13

// PRESET BOT CONFIGURATIONS
constexpr bot_config_t botConfigArray[NUM_BOTS] = {
// idx  bot_name    bot_type     motors
  { 0,  "i++",      lineman,     small  },  // 0:  i++
  { 1,  "sqrt(-1)", lineman,     small  },  // 1:  sqrt(-1)
  { 2,  "pi",       lineman,     small  },  // 2:  pi
  { 3,  "rho",      lineman,     small  },  // 3:  ρ
  { 4,  "2.72",     lineman,     big    },  // 4:  2.72
  { 5,  ":)",       lineman,     small  },  // 5:  :)
  { 6,  ">=",       lineman,     big    },  // 6:  >=
  { 7,  "32.2",     receiver,    small  },  // 7:  32.2
  { 8,  "9.8",      receiver,    big    },  // 8:  9.8
  { 9,  "c",        runningback, falcon },  // 9:  c
  { 10, "phi",      center,      small  },  // 10: Φ
  { 11, "inf",      quarterback, small  },  // 11: ∞
  { 12, "theta",    kicker,      small  }   // 12: Θ
};

const char * getBotTypeString(BotType type);
const char * getBotName(uint8_t index);

#endif // BOT_TYPES_H
