#ifndef _READ_CONGIG_H_
#define _READ_CONFIG_H_

#include <Arduino.h>
#include <Utilities/BotTypes.h>


class ReadConfig {
private:
    Preferences preferences;
    botconfig_t* config;
    // uint8_t m_bot_index;
    // eBOT_TYPE m_bot_type;
    const char * botNameToString(uint8_t n_idx);
    const char * BotTypeToString(eBOT_TYPE bot);
    const char * MotorTypeToString(eMOTOR_TYPE mot);
public:
    ReadConfig();
    void read();
    int BotIdx();
    eBOT_TYPE BotType();
    eMOTOR_TYPE MotType();
    // int GearRatio()
    const char * toString();
};

#endif