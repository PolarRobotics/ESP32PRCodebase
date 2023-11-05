#pragma once

#ifndef BOT_TYPES_H
#define BOT_TYPES_H

#include <Arduino.h>
#include <Utilities/Pair.h>
#include <Utilities/MotorTypes.h>
#include <Utilities/DriveParameters.h>

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
  const char* bot_name;
  BotType bot_type; // primary robot position
  MotorType mot_type;
  drive_param_t drive_params;
  // float gear_ratio;
  // float wheel_base;
  // LED STRIP:
  // bool has_leds;
  // uint8_t num_leds;
  // placeholder: fHasMultipleBotTypes (for new linemen/receivers)
  // BotType secondary_type;
} bot_config_t;

#define NUM_BOTS 13

// PRESET BOT CONFIGURATIONS, MUST MATCH:
// https://docs.google.com/spreadsheets/d/1DswoEAcry9L9t_4ouKL3mXFgDMey4KkjEPFXULQxMEQ/edit#gid=0
constexpr bot_config_t botConfigArray[NUM_BOTS] = {
// idx  bot_name    bot_type     motor_type      gear_ratio wheel_base r_min   r_max
  { 0,  "i++",      lineman,     small_ampflow,  { 0.6f,      12.25f,  9.00f,  36.00f }},  // 0:  i++
  { 1,  "sqrt(-1)", lineman,     big_ampflow,    { 0.53333f,  11.25f,  9.00f,  36.00f }},  // 1:  sqrt(-1)
  { 2,  "pi",       lineman,     small_ampflow,  { 0.46667f,  11.00f,  9.00f,  36.00f }},  // 2:  pi
  { 3,  "rho",      lineman,     big_ampflow,    { 0.6f,      11.25f,  9.00f,  36.00f }},  // 3:  ρ
  { 4,  "2.72",     lineman,     big_ampflow,    { 0.4f,      11.25f,  9.00f,  36.00f }},  // 4:  2.72
  { 5,  ":)",       lineman,     small_ampflow,  { 1.0f,      9.75f ,  9.00f,  36.00f }},  // 5:  :)
  { 6,  ">=",       lineman,     big_ampflow,    { 1.0f,      10.00f,  6.00f,  27.00f }},  // 6:  >=
  { 7,  "32.2",     receiver,    small_ampflow,  { 0.5f,      11.50f,  9.00f,  36.00f }},  // 7:  32.2
  { 8,  "9.8",      receiver,    big_ampflow,    { 0.5f,      11.50f,  9.00f,  36.00f }},  // 8:  9.8
  { 9,  "c",        runningback, falcon,         { 0.5f,      8.00f ,  9.00f,  36.00f }},  // 9:  c
  { 10, "phi",      center,      small_ampflow,  { 0.6f,      11.50f,  9.00f,  36.00f }},  // 10: Φ
  { 11, "inf",      quarterback, small_ampflow,  { 0.5625f,   11.50f,  9.00f,  36.00f }},  // 11: ∞
  { 12, "theta",    kicker,      small_ampflow,  { 0.34375f,  10.00f,  9.00f,  36.00f }}   // 12: Θ
};

const char * getBotTypeString(BotType type);
const char * getBotName(uint8_t index);

#endif // BOT_TYPES_H
