#pragma once

#ifndef _CFG_MGR_H_
#define _CFG_MGR_H_

#include <Arduino.h>
#include <Preferences.h>

// #include <Utilities/BotTypes.h>
#include <PolarRobotics.h>

#if CFG_WRITABLE == 1
  #define DEFAULT_WRITABLE 1
#else
  #define DEFAULT_WRITABLE 0
#endif

class ConfigManager {
  private:
    bool writable = DEFAULT_WRITABLE;
    uint8_t bot_name_index;
    eBOT_TYPE bot_type;
    eMOTOR_TYPE motor_type;
    bool write(bot_config_t *cfg);
    bot_config_t* config;
  protected:
    Preferences preferences; //! Do we know if having more than one instance of Preferences is OK? - MP
  public:
    ConfigManager();
    // ConfigManager(bool writable = false);
    ~ConfigManager();
    void read();
    int botIndex();
    eBOT_TYPE getBotType();
    eMOTOR_TYPE getMotorType();
    // int GearRatio()
    const char * toString();
    bool setConfig(uint8_t botindex);
    bool setConfig(uint8_t botindex, eBOT_TYPE bottype, eMOTOR_TYPE motortype);
};

#endif