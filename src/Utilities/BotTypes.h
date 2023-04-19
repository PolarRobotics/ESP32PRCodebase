#ifndef A5BF803D_491D_477A_8854_A7D87F1E0251
#define A5BF803D_491D_477A_8854_A7D87F1E0251

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
  eBOT_TYPE bot_type;
  eMOTOR_TYPE mot_type;
  // placeholder gear ratio index
};

typedef const struct botconfig botconfig_t;
// BOT CONFIGURATIONS
botconfig_t bot_config_arr[NUM_BOTS] = {
    {0,  lineman, small},  // 0:  i++
    {1,  lineman, small},  // 1:  sqrt(-1)
    {2,  lineman, small},  // 2:  pi
    {3,  lineman, small},  // 3:  p
    {4,  lineman, small},  // 4:  2.72
    {5,  lineman, small},  // 5:  :)
    {6,  lineman, small},  // 6:  >=
    {7,  lineman, small},  // 7:  32.2
    {8,  lineman, small},  // 8:  9.8
    {9,  lineman, small},  // 9:  c
    {10, lineman, small},  // 10: 0
    {11, lineman, small},  // 11: inf
    {12, lineman, small}   // 12: theta
};

// enum eBOT_NAME {
//   iplusplus,            // i++
//   sqrt_neg_one,         // sqrt(-1)
//   pi,                   // pi
//   rho,                  // p
//   two_point_seven_two,  // 2.72
//   smiley,               // :)
//   greaterthanequal,     // >=
//   thirty_two_point_two, // 32.2
//   nine_point_eight,     // 9.8
//   c,                    // c
//   phi,                  // 0
//   infinity,             // inf
//   theta                 // theta
// };

#endif /* A5BF803D_491D_477A_8854_A7D87F1E0251 */
