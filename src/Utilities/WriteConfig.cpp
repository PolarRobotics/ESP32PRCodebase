#include "WriteConfig.h"

WriteConfig::WriteConfig() {
    // this->custom = custom_config;
    this->config = new botconfig_t();
}

WriteConfig::~WriteConfig() {
    delete this->config;
}

/**
 * @brief write2Cfg writes the configuration passed into the function to the pseudo EEPROM 
 * 
 * the bot configuration is stored in the "bot_config" "namespace" for preferences,
 * this is a grouping of members, anything under the given namespace is grouped under said namespace
 * data is accessed by using a key, which is basically a pointer to the location of the 
 * data you are trying to read/write 
 * 
 * @param cfg the configuration you wish to save to EEPROM
 */
void WriteConfig::write2Cfg(botconfig_t *cfg) {
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


bool WriteConfig::setConfig(uint8_t botindex) {
    // validate bot index
    if (botindex < 0 || botindex > (NUM_BOTS - 1)) return false;
    
    // write index to predefined configuration from the array defined in the header file
    write2Cfg(bot_config_arr[botindex]);
    return true;
}

bool WriteConfig::setConfig(uint8_t botindex, eBOT_TYPE bottype, eMOTOR_TYPE motortype) {
    // validate bot index
    // if (botindex < 0 || botindex > (NUM_BOTS - 1)) return false;
    this->config->index = botindex;
    this->config->bot_type = bottype;
    this->config->mot_type = motor_type; 
    
    write2Cfg(this->config);
    return true;
}