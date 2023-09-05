#include "ConfigManager.h"

using namespace std;

ConfigManager::ConfigManager() {
  this->config = new bot_config_t();
}

// ConfigManager::ConfigManager(bool writable) {
//   this->writable = writable;
//   this->config = new bot_config_t();
// }

ConfigManager::~ConfigManager() {
  delete this->config;
}

/**
 * @brief reads the bot properties from the preferences or pseudo eeprom
 * and stores those properties to the config data structure, 
 * which can be accessed through getters.
 * 
 * the bot configuration is stored in the "bot_config" "namespace" for preferences,
 * this is a grouping of members, anything under the given namespace is grouped under said namespace
 * data is accessed by using a key, which is basically a pointer to the location of the 
 * data you are trying to read/write  
 */
void ConfigManager::read() {
    // open the bot_config namespace, readonly is true (defaults to false)
    preferences.begin("bot_config", true);
    // read the bot name index   
    this->config->index = preferences.getUChar("bot_name_idx");
    // read the bot type
    this->config->bot_type = (eBOT_TYPE)preferences.getUChar("bot_type");
    // read the motor type
    this->config->mot_type = (eMOTOR_TYPE)preferences.getUChar("motor_type");
    // close the bot_config namespace
    preferences.end();
}

/**
 * @brief gets the bot index from the configuration.
 * this index corresponds to the position in the config array that the bot config is pulled from
 * @return int the bot index in the array the configuration is from 
 */
int ConfigManager::botIndex() {
    return this->config->index;
}

/**
 * @brief getBotType gets the bot type (linemen, receiver, runningback, etc...) from the configuration
 * @return eBOT_TYPE the stored bot type enumeration
 */
eBOT_TYPE ConfigManager::getBotType() {
    return this->config->bot_type;
}

/**
 * @brief getMotorType gets the motor type (small, bit, mecaummotor) from the configuration
 * @return eMOTOR_TYPE the stored motor type enumeration
 */
eMOTOR_TYPE ConfigManager::getMotorType() {
    return this->config->mot_type;
}

/**
 * @brief converts the stored configuration into an easy to read string, 
 * to be printed to the console or other means.
 * 
 * intended for for easy debugging
 * 
 * @return const char* the string containing the bot configuration
 */
const char * ConfigManager::toString() {
    string temp = "\nBot info: ";
    temp.append("\nbot array index #: ");
    temp.append(to_string(config->index));
    temp.append("\nbot name: ");
    temp.append(getBotName(config->index));
    temp.append("\nbot type: ");
    temp.append(getBotTypeString(config->bot_type));
    temp.append("\nmotor type: ");
    temp.append(getMotorTypeString(config->mot_type));
    temp.append("\r\n");
    return temp.c_str();
}

//! Writable Methods Below

/**
 * @brief writes the configuration passed into the function to the pseudo EEPROM 
 * 
 * the bot configuration is stored in the "bot_config" "namespace" for preferences,
 * this is a grouping of members, anything under the given namespace is grouped under said namespace
 * data is accessed by using a key, which is basically a pointer to the location of the 
 * data you are trying to read/write 
 * 
 * @param cfg the configuration you wish to save to EEPROM
 * @return true if this instance of the config manager is writable, false if not
 */
bool ConfigManager::write(bot_config_t *cfg) {
  if (this->writable) {  
    // open the "bot_config" namespace and set it to read/write
    bool good = preferences.begin("bot_config", false); 
    if (!good) return false;
    // store the bot name to preferences
    preferences.putUChar("bot_name_idx", cfg->index);

    // store the bot type to preferences
    preferences.putUChar("bot_type", static_cast<uint8_t>(cfg->bot_type));
  
    // store the bot type to preferences
    preferences.putUChar("motor_type", static_cast<uint8_t>(cfg->mot_type));

    // close the namespace
    preferences.end();
    return true;
  } else return false;
}

/**
 * @brief sets the preset configuration from botConfigArray 
 * to be stored to the configuration and writes it to EEPROM 
 * 
 * @param botindex index of the preset configuration array to set to the bot
 * @return true if configuration was successfully applied;
 * @return false if configuration check failed or index out of range of array
 */
bool ConfigManager::setConfig(uint8_t botindex) {
    // validate bot index
    if (botindex < 0 || botindex > (NUM_BOTS - 1) || !this->writable) return false;
    
    this->config->index = botConfigArray[botindex].index;
    this->config->bot_type = botConfigArray[botindex].bot_type;
    this->config->mot_type = botConfigArray[botindex].mot_type;

    // write index to predefined configuration from the array defined in the header file
    return write(this->config);
}

/**
 * @brief setConfig writes a custom configuration to the bot's EEPROM,
 * overriding previous preset configurations
 * 
 * @param botindex not entirely relevant, but the index of the bot in the array
 * @param bottype the type of bot you wish to configure
 * @param motortype the motor type you wish to assign to the bot
 * @return true configuration was successfully applied
 * @return false configuration check failed
 */
bool ConfigManager::setConfig(uint8_t botindex, eBOT_TYPE bottype, eMOTOR_TYPE motortype) {
  if (this->writable) {
    this->config->index = botindex;
    this->config->bot_type = bottype;
    this->config->mot_type = motor_type; 
    
    return write(this->config);
  } else return false;
}