#include "ConfigManager.h"

using namespace std;

#define CODE_VERSION_PREF_KEY "version"
#define NO_VERSION_PLACEHOLDER "NO_VERSION"

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
    this->config->bot_type = (BotType)preferences.getUChar("bot_type");
    // read the motor type
    this->config->mot_type = (MotorType)preferences.getUChar("motor_type");
    // read the gear ratio
    this->config->drive_params.gear_ratio = (float)preferences.getFloat("gear_ratio");
    // read the wheel base
    this->config->drive_params.wheel_base = (float)preferences.getFloat("wheel_base");
    // read the r min
    this->config->drive_params.r_min = (float)preferences.getFloat("r_min");
    // read the r max
    this->config->drive_params.r_max = (float)preferences.getFloat("r_max");


    // close the bot_config namespace
    preferences.end();
}

/**
 * @brief gets the code version from the configuration.
 * @return String - the version as a string
 */
String ConfigManager::version() {
  bool good = preferences.begin(CODE_VERSION_PREF_KEY, true);
  if (!good) return NO_VERSION_PLACEHOLDER;
  String str = preferences.getString(CODE_VERSION_PREF_KEY, NO_VERSION_PLACEHOLDER);
  preferences.end();
  return str;
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
 * @return BotType the stored bot type enumeration
 */
BotType ConfigManager::getBotType() {
    return this->config->bot_type;
}

/**
 * @brief getMotorType gets the motor type (small, bit, mecaummotor) from the configuration
 * @return MotorType the stored motor type enumeration
 */
MotorType ConfigManager::getMotorType() {
    return this->config->mot_type;
}

/**
 * @brief getGearRatio returns the gear ratio (input tooth count / output tooth count)
 * @return float the stored gear ratio decimal
*/
// float ConfigManager::getGearRatio() {
//     return this->config->gear_ratio;
// }

/**
 * @brief getWheelBase returns the wheelbase the distance between the centers of the two drive tires
 * for a standard two-wheel differential drive bot
 * 
 * @return float stored wheelbase value
*/
// float ConfigManager::getWheelBase() {
//     return this->config->wheel_base;
// }

drive_param_t ConfigManager::getDriveParams() {
    return this->config->drive_params;
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
    temp.append("\ncode version: ");
    temp.append(version().c_str());
    temp.append("\nbot array index #: ");
    temp.append(to_string(config->index));
    temp.append("\nbot name: ");
    temp.append(getBotName(config->index));
    temp.append("\nbot type: ");
    temp.append(getBotTypeString(config->bot_type));
    temp.append("\nmotor type: ");
    temp.append(getMotorTypeString(config->mot_type));
    temp.append("\ngear ratio: ");
    temp.append(to_string(config->drive_params.gear_ratio));
    temp.append("\nwheel base: ");
    temp.append(to_string(config->drive_params.wheel_base));
    temp.append("\nr_min: ");
    temp.append(to_string(config->drive_params.r_min));
    temp.append("\nr_max: ");
    temp.append(to_string(config->drive_params.r_max));
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
bool ConfigManager::write(bot_config_t* cfg) {
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
    // store the robots drivetrain gear ratio
    preferences.putFloat("gear_ratio", cfg->drive_params.gear_ratio);
    // store the robots drivetrain wheel base
    preferences.putFloat("wheel_base", cfg->drive_params.wheel_base);
    // store the robots drivetrain r min
    preferences.putFloat("r_min", cfg->drive_params.r_min);
    // store the robots drivetrain r max
    preferences.putFloat("r_max", cfg->drive_params.r_max);

    // close the namespace
    preferences.end();

    // store version to preferences

    good = preferences.begin(CODE_VERSION_PREF_KEY, false); 
    if (!good) return false;
    preferences.putString(CODE_VERSION_PREF_KEY, PR_CODEBASE_VERSION);
    preferences.end();

    return true;
  } else return false;
}

/**
 * @brief sets the preset configuration from botConfigArray 
 * to be stored to the configuration and writes it to EEPROM 
 * 
 * @param botIndex index of the preset configuration array to set to the bot
 * @return true if configuration was successfully applied;
 * @return false if configuration check failed or index out of range of array
 */
bool ConfigManager::setConfig(uint8_t botIndex) {
    // validate bot index
    if (botIndex < 0 || botIndex > (NUM_BOTS - 1) || !this->writable) return false;
    
    this->config->index = botConfigArray[botIndex].index;
    this->config->bot_type = botConfigArray[botIndex].bot_type;
    this->config->mot_type = botConfigArray[botIndex].mot_type;
    this->config->bot_name = botConfigArray[botIndex].bot_name;
    this->config->drive_params.gear_ratio = botConfigArray[botIndex].drive_params.gear_ratio;
    this->config->drive_params.wheel_base = botConfigArray[botIndex].drive_params.wheel_base;
    this->config->drive_params.r_min = botConfigArray[botIndex].drive_params.r_min;
    this->config->drive_params.r_max = botConfigArray[botIndex].drive_params.r_max;

    // write index to predefined configuration from the array defined in the header file
    return write(this->config);
}

/**
 * @brief setConfig writes a custom configuration to the bot's EEPROM,
 * overriding previous preset configurations
 * 
 * @param botindex not entirely relevant, but the index of the bot in the array to be assigned
 * @param bottype the type of bot you wish to configure
 * @param motortype the motor type you wish to assign to the bot
 * @param gearratio the gear ratio you wish to assign to the bot
 * @param wheelbase the distance between drive wheels
 * @param rmin the minimum turning radius the robot can achieve
 * @param rmax the maximum turning radius the robot can achieve  
 * @return true configuration was successfully applied
 * @return false configuration check failed
 */
bool ConfigManager::setConfig(uint8_t botindex, BotType bottype, MotorType motortype, float gearratio, float wheelbase, float rmin, float rmax) {
  if (this->writable) {
    this->config->index = botindex;
    this->config->bot_type = bottype;
    this->config->mot_type = motortype;
    this->config->bot_name = "Custom Robot";
    this->config->drive_params.gear_ratio = gearratio;
    this->config->drive_params.wheel_base = wheelbase;
    this->config->drive_params.r_min = rmin;
    this->config->drive_params.r_max = rmax;
    
    return write(this->config);
  } else return false;
}