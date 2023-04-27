#pragma once

#ifndef _BOT_TYPES_H_
#define _BOT_TYPES_H_

#include <Arduino.h>
#include <map>
#include <Utilities/MotorTypes.h>

#define NUM_BOTS 13 // used in BotTypes.h

/** eBOT_TYPE
 * enum for the possible positions a robot can have on the field
 * NOTE: when this list is updated, make sure to update the bot_type_arr in BotTypes.h
 * also please keep count at the end of the list, if you need to add anything, add it before count
 * 
 *  Robot Type Enum
 *  Lineman: 0
 *  Receiver: 1
 *  Runningback: 2
 *  Center: 3
 *  Mecanum Center: 4
 *  QuarterBack: 5
 *  Kicker: 6
*/
#define NUM_POSITIONS 7
typedef enum {
  lineman,
  receiver,
  runningback,
  center,
  mecanum_center,
  quarterback,
  kicker
} eBOT_TYPE;


/**
 * @brief botconfig robot configuration datastructure, 
 * used to read and write bot information to the esp 
 * Order to write to eeprom:
 * Bot Name index
 * Bot Type enum
 * Motor Type enum
 * Gear Ratio index
 * ? code last uploaded date and time
 */

struct botconfig {
    uint8_t index;
    const char * bot_name;
    eBOT_TYPE bot_type; // primary robot position
    eMOTOR_TYPE mot_type;
    // placeholder: gear ratio index
    // uint8_t GR_index;
    // LED STRIP:
    // bool has_leds;
    // uint8_t num_leds;
    // placeholder: fHasMultipleBotTypes (for new linemen/receivers)
    // eBOT_TYPE secondary_type;
};

typedef struct botconfig botconfig_t;

class BotTypes {
protected:
    // PRESET BOT CONFIGURATIONS
    const botconfig_t bot_config_arr[NUM_BOTS] = {
        {0,  "i++",      lineman,     small},  // 0:  i++
        {1,  "sqrt(-1)", lineman,     small},  // 1:  sqrt(-1)
        {2,  "pi",       lineman,     small},  // 2:  pi
        {3,  "rho",      lineman,     small},  // 3:  p
        {4,  "2.72",     lineman,     big  },  // 4:  2.72
        {5,  ":)",       lineman,     small},  // 5:  :)
        {6,  ">=",       lineman,     big  },  // 6:  >=
        {7,  "32.2",     receiver,    big  },  // 7:  32.2
        {8,  "9.8",      receiver,    small},  // 8:  9.8
        {9,  "c",        runningback, small},  // 9:  c
        {10, "phi",      center,      small},  // 10: 0
        {11, "inf",      quarterback, small},  // 11: inf
        {12, "theta",    kicker,      small}   // 12: theta
    };
    
    const char * bot_type_string_arr[NUM_POSITIONS] = {
        "lineman", 
        "receiver", 
        "runningback", 
        "center", 
        "mecanum_center",
        "quarterback",
        "kicker"
    };

    const char * motor_type_string_arr[NUM_MOTOR_TYPES] = {
        "big", "small", "mecanum_motor", "falcon_motor"
    };
};

#endif /* _BOT_TYPES_H_ */
