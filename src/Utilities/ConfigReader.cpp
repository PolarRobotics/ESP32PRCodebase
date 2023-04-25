#include "ConfigReader.h"

using namespace std;

ConfigReader::ConfigReader() {
    this->config = new botconfig_t();
}

ConfigReader::~ConfigReader() {
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
void ConfigReader::read() {
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
int ConfigReader::BotIdx() {
    return this->config->index;
}

/**
 * @brief BotType gets the bot type (linemen, receiver, runningback, etc...) from the configuration
 * @return eBOT_TYPE the stored bot type enumeration
 */
eBOT_TYPE ConfigReader::BotType() {
    return this->config->bot_type;
}

/**
 * @brief MotType gets the motor type (small, bit, mecaummotor) from the configuration
 * @return eMOTOR_TYPE the stored motor type enumeration
 */
eMOTOR_TYPE ConfigReader::MotType() {
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
const char * ConfigReader::toString() {
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

/**
 * @brief botNameToString uses the index of the bot, to index the array, defined in BotTypes.h
 * if the bot index is out of range of the number of possible bots, returns "Robot"
 * 
 * @param n_idx the index of the bot in the array
 * @return const char* if the index is within range return the bot name
 */
const char * ConfigReader::botNameToString(uint8_t n_idx) {
    return (n_idx > 0 && n_idx < NUM_BOTS - 1) ? bot_name_arr[n_idx] : "Robot"; 
}

/**
 * @brief BotTypeToString returns a string correlating to the position defined in eeprom,
 * string array defined in BotTypes.h
 * for example if the bot positon enum is linemen the function returns "linemen"
 * 
 * @param bot 
 * @return const char* 
 */
const char * ConfigReader::BotTypeToString(eBOT_TYPE bot) {
    return bot_type_string_arr[static_cast<uint8_t>(bot)]; 
}

/**
 * @brief MotorTypeToString returns a string correlating to the motor type defined in eeprom,
 * string array defined in BotTypes.h
 * for example: if the motor type enum is falconmotor the function returns "falconmotor"
 * 
 * @param mot the enum to be converted to string
 * @return const char* the string containing the name of the motor type
 */
const char * ConfigReader::MotorTypeToString(eMOTOR_TYPE mot) {
    return motor_type_string_arr[static_cast<uint8_t>(mot)]; 
}
