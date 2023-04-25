#pragma once

#ifndef _BOT_TYPES_H_
#define _BOT_TYPES_H_

#include <Arduino.h>
#include <PolarRobotics.h>

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
    
    const char * bot_type_string_arr[static_cast<int>(eBOT_TYPE::positions_count)] = {
        "lineman", 
        "receiver", 
        "runningback", 
        "center", 
        "mecanum_center",
        "quarterback",
        "kicker"
    };

    const char * motor_type_string_arr[static_cast<int>(eMOTOR_TYPE::motor_type_count)] = {
        "big", "small", "mecanummotor", "falonmotor"
    };
};

#endif /* _BOT_TYPES_H_ */
