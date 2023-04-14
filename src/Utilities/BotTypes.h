#ifndef A5BF803D_491D_477A_8854_A7D87F1E0251
#define A5BF803D_491D_477A_8854_A7D87F1E0251

#include <Arduino.h>
#include <PolarRobotics.h>
#include <string>

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


typedef const struct botconfig {
  uint8_t index;
  eBOT_TYPE bot_type;
  eMOTOR_TYPE mot_type;
  // placegolder gear ratio index
} botconfig_t;

// BOT CONFIGURATIONS
botconfig_t bot_config_arr[NUM_BOTS] = {
    {0,  linemen, small},  // 0
    {1,  linemen, small},  // 1
    {2,  linemen, small},  // 2
    {3,  linemen, small},  // 3
    {4,  linemen, small},  // 4
    {5,  linemen, small},  // 5
    {6,  linemen, small},  // 6
    {7,  linemen, small},  // 7
    {8,  linemen, small},  // 8
    {9,  linemen, small},  // 9
    {10, linemen, small},  // 10
    {11, linemen, small},  // 11
    {12, linemen, small},  // 12
    {13, linemen, small},  // 13
};

/***
 * read the bot info from eeprom
*/
class BotTypes {
private:
    botconfig_t config;
    // uint8_t m_bot_index;
    // eBOT_TYPE m_bot_type;
public:
    BotTypes();
    void readBotInfo();
    const char* botName2_str(uint8_t n_idx);

}


/**
 * @brief BotConfigurator
 * used for writing the bot info to the eeprom
 * inherits bottypes
 */

class BotConfigurator : public BotTypes {
private:
public: 

    string botName2_str(uint8_t n_idx);

}

#endif /* A5BF803D_491D_477A_8854_A7D87F1E0251 */
