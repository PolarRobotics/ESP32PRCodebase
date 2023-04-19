#ifndef A5BF803D_491D_477A_8854_A7D87F1E0251
#define A5BF803D_491D_477A_8854_A7D87F1E0251

#include <Arduino.h>
#include <PolarRobotics.h>
// #include <string>

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


/**
 * @brief 
 * 
 * 
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
    {0,  eBOT_TYPE::lineman, eMOTOR_TYPE::small},  // 0
    {1,  eBOT_TYPE::lineman, eMOTOR_TYPE::small},  // 1
    {2,  eBOT_TYPE::lineman, eMOTOR_TYPE::small},  // 2
    {3,  eBOT_TYPE::lineman, eMOTOR_TYPE::small},  // 3
    {4,  eBOT_TYPE::lineman, eMOTOR_TYPE::small},  // 4
    {5,  eBOT_TYPE::lineman, eMOTOR_TYPE::small},  // 5
    {6,  eBOT_TYPE::lineman, eMOTOR_TYPE::small},  // 6
    {7,  eBOT_TYPE::lineman, eMOTOR_TYPE::small},  // 7
    {8,  eBOT_TYPE::lineman, eMOTOR_TYPE::small},  // 8
    {9,  eBOT_TYPE::lineman, eMOTOR_TYPE::small},  // 9
    {10, eBOT_TYPE::lineman, eMOTOR_TYPE::small},  // 10
    {11, eBOT_TYPE::lineman, eMOTOR_TYPE::small},  // 11
    {12, eBOT_TYPE::lineman, eMOTOR_TYPE::small},  // 12
    {13, eBOT_TYPE::lineman, eMOTOR_TYPE::small},  // 13
};

/***
 * read the bot info from eeprom
*/
class BotTypes {
private:
    botconfig_t* config;
    // uint8_t m_bot_index;
    // eBOT_TYPE m_bot_type;
public:
    BotTypes();
    void readBotInfo();
    const char * botNameToString(uint8_t n_idx);

}


// /**
//  * @brief BotConfigurator
//  * used for writing the bot info to the eeprom
//  * inherits bottypes
//  */

// class BotConfigurator : public BotTypes {
// private:
// public: 

//     string botName2_str(uint8_t n_idx);

// }

#endif /* A5BF803D_491D_477A_8854_A7D87F1E0251 */
