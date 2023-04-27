#pragma once

#ifndef _READ_CONGIG_H_
#define _READ_CONFIG_H_

#include <Arduino.h>
#include <Preferences.h>

// #include <Utilities/BotTypes.h>
#include <PolarRobotics.h>


class ConfigReader : public BotTypes {
private:
    const char * botNameToString(uint8_t n_idx);
    const char * MotorTypeToString(eMOTOR_TYPE mot);
protected:
    bot_config_t* config;
    Preferences preferences;
public:
    ConfigReader();
    ~ConfigReader();
    void read();
    int BotIdx();
    eBOT_TYPE BotType();
    eMOTOR_TYPE MotType();
    // int GearRatio()
    const char * toString();
};

#endif