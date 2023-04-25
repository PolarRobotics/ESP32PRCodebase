#pragma once

#ifndef _READ_CONFIG_H_
#define _READ_CONFIG_H_

#include <Arduino.h>
#include <Preferences.h>

#include <Utilities/ConfigReader.h>
#include <Utilities/BotTypes.h>

class ConfigWriter : public ConfigReader {
private:
    // bool custom; // is the bot listed in the array above?
    uint8_t bot_name_index;
    eBOT_TYPE bot_type;
    eMOTOR_TYPE motor_type;
    void write2EEPROM(botconfig_t *cfg);
public:
    ConfigWriter();
    ~ConfigWriter();
    bool setConfig(uint8_t botindex);
    bool setConfig(uint8_t botindex, eBOT_TYPE bottype, eMOTOR_TYPE motortype);
};

#endif /* _READ_CONFIG_H_ */
