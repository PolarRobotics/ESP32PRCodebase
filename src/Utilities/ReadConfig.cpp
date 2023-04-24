#include "ReadConfig.h"

using namespace std;

ReadConfig::ReadConfig() {
    this->config = new botconfig_t();
}

ReadConfig::~ReadConfig() {
    delete this->config;
}

/**
 * @brief read reads the bot properties from the preferences or pseudo eeprom
 * and stores those properties to the config data structure, 
 * which can be accessed through getters
 * 
 * the bot configuration is stored in the "bot_config" "namespace" for preferences,
 * this is a grouping of members, anything under the given namespace is grouped under said namespace
 * data is accessed by using a key, which is basically a pointer to the location of the 
 * data you are trying to read/write  
 */
void ReadConfig::read() {
    // open the bot_config namespace, readonly is true (defaults to false)
    preferences.begin("bot_config", true);
    // read the bot name index   
    this->config->index = preferences.getUChar("bot_name_idx");
    // read the bot type
    this->config->bot_type = (eBOT_TYPE)preferences.getUChar("bot_type");
    // read the bot type
    this->config->mot_type = (eMOTOR_TYPE)preferences.getUChar("motor_type");
    // close the bot_config namespace
    preferences.end();
}

/**
 * @brief BotIdx gets the bot index from the configuration,
 * this index corresponds to the position in the config array that the bot config is pulled from
 * @return int the bot index in the array the configuration is from 
 */
int ReadConfig::BotIdx() {
    return this->config->index;
}

/**
 * @brief BotType gets the bot type (linemen, receiver, runningback, etc...) from the configuration
 * @return eBOT_TYPE the stored bot type enumeration
 */
eBOT_TYPE ReadConfig::BotType() {
    return this->config->bot_type;
}

/**
 * @brief MotType gets the motor type (small, bit, mecaummotor) from the configuration
 * @return eMOTOR_TYPE the stored motor type enumeration
 */
eMOTOR_TYPE ReadConfig::MotType() {
    return this->config->mot_type;
}

/**
 * @brief toString
 * converts the stored configuration into an easy to read string, 
 * to be printed to the console or other means
 * 
 * intended for for easy debugging
 * 
 * @return const char* the string containing the bot configuration
 */
const char * ReadConfig::toString() {
    string temp = "\nBot info: ";
    temp.append("\nbot array index #: ");
    temp.append(to_string(config->index));
    temp.append("\nbot name: ");
    temp.append(botNameToString(config->index));
    temp.append("\nbot type: ");
    temp.append(BotTypeToString(config->bot_type));
    temp.append("\nmotor type: ");
    temp.append(MotorTypeToString(config->mot_type));
    temp.append("\r\n");
    return temp.c_str();
}

const char * ReadConfig::botNameToString(uint8_t n_idx) {
    switch(n_idx) {
        case 0:  return "i++";
        case 1:  return "sqrt(-1)";
        case 2:  return "pi";
        case 3:  return "rho";
        case 4:  return "2.72";
        case 5:  return ":)";
        case 6:  return ">=";
        case 7:  return "32.2";
        case 8:  return "9.8";
        case 9:  return "c";
        case 10: return "phi";
        case 11: return "inf";
        case 12: return "theta";
        default: return "Robot"; 
    }
}

const char * ReadConfig::BotTypeToString(eBOT_TYPE bot) {
    switch (bot) {
        case lineman:        return "lineman";
        case receiver:       return "receiver";
        case runningback:    return "runningback";
        case center:         return "center";
        case mecanum_center: return "mecanum_center";
        case quarterback:    return "quarterback";
        case kicker:         return "kicker";
        default:             return "Robot";
    }
}

const char * ReadConfig::MotorTypeToString(eMOTOR_TYPE mot) {
    switch (mot) {
        case big: return "big";
        case small: return "small";
        case mecanummotor: return "mecanummotor";
        case falconmotor: return "falconmotor";
        default: return "big";
    }
}

// string BotTypes::botType2String(uint8_t t_idx) {
//   switch (t_idx) {
//     case 0: return "lineman";
//     case 1: return "receiver";
//     case 2: return "runningback";
//     case 3: return "center";
//     case 4: return "mecanum_center";
//     case 5: return "quarterback";
//     case 6: return "kicker";
//     default: return "incorrect Bot Type declared";
//   }
// }