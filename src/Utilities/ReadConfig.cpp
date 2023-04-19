
#include <Arduino.h>
#include <string.h>
#include <Preferences.h>

#include "Utilities/ReadConfig.h"
#include "ReadConfig.h"
// #include "ReadConfig.h"

using namespace std;

ReadConfig::ReadConfig() {
    this->config = new botconfig_t();
}

void ReadConfig::read() {
    // open the bot_config namespace
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
eBOT_TYPE ReadConfig::BotType() {
    return this->config->bot_type;
}

eMOTOR_TYPE ReadConfig::MotType() {
    return this->config->mot_type;
}

const char *ReadConfig::toString() {
    string temp = "Bot info:
    \nbot index #: " + config->index + 
    "\nbot name: " + botNameToString(config->index) +
    "\nbot type: " + BotTypeToString(config->bot_type) +
    "\nmotor type: " + MotorTypeToString(config->mot_type);
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
    }
}

const char *ReadConfig::MotorTypeToString(eMOTOR_TYPE mot) {
    switch (mot) {
        case big: return "big";
        case small: return "small";
        case mecanummotor: return "mecanummotor";
        case falconmotor: return "falconmotor";
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