#include "WriteConfig.h"

WriteConfig::WriteConfig() {
    // this->custom = custom_config;
    this->config = new botconfig_t();
}

WriteConfig::~WriteConfig() {
    delete this->config;
}

/**
 * @brief write2EEPROM writes the configuration passed into the function to the pseudo EEPROM 
 * 
 * the bot configuration is stored in the "bot_config" "namespace" for preferences,
 * this is a grouping of members, anything under the given namespace is grouped under said namespace
 * data is accessed by using a key, which is basically a pointer to the location of the 
 * data you are trying to read/write 
 * 
 * @param cfg the configuration you wish to save to EEPROM
 */
void WriteConfig::write2EEPROM(botconfig_t *cfg) {
    // open the "bot_config" namespace and set it to read/write
    preferences.begin("bot_config", false); 
    // store the bot name to preferences
    preferences.putUChar("bot_name_idx", cfg->index);

    // store the bot type to preferences
    preferences.putUChar("bot_type", static_cast<uint8_t>(cfg->bot_type));
  
    // store the bot type to preferences
    preferences.putUChar("motor_type", static_cast<uint8_t>(cfg->mot_type));

    // close the namespace
    preferences.end();
}

/**
 * @brief setConfig sets the configuration to be stored to the 
 * configuration defined in the preset bot_config_arr array 
 * 
 * @param botindex index of the preset configuration array to set to the bot
 * @return true configuration was successfully applied
 * @return false configuration check failed, index out of range of array
 */
bool WriteConfig::setConfig(uint8_t botindex) {
    // validate bot index
    if (botindex < 0 || botindex > (NUM_BOTS - 1)) return false;
    
    this->config->index = bot_config_arr[botindex].index;
    this->config->bot_type = bot_config_arr[botindex].bot_type;
    this->config->mot_type = bot_config_arr[botindex].mot_type;

    // write index to predefined configuration from the array defined in the header file
    write2EEPROM(this->config);
    return true;
}

/**
 * @brief setConfig sets a custom configuration to the bot, 
 * overriding previous preset configurations
 * 
 * @param botindex not entirely relevant, but the index of the bot in the array
 * @param bottype the type of bot you wish to configure
 * @param motortype the motor type you wish to assign to the bot
 * @return true configuration was successfully applied
 * @return false configuration check failed
 */
bool WriteConfig::setConfig(uint8_t botindex, eBOT_TYPE bottype, eMOTOR_TYPE motortype) {
    // validate bot index
    // if (botindex < 0 || botindex > (NUM_BOTS - 1)) return false;
    this->config->index = botindex;
    this->config->bot_type = bottype;
    this->config->mot_type = motor_type; 
    
    write2EEPROM(this->config);
    return true;
}