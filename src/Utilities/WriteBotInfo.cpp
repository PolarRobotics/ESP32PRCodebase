#include <Arduino.h>
// #include <Preferences.h>

// #include <PolarRobotics.h>
#include <Utilities/ConfigManager.h>

// Preferences preferences;

ConfigManager config;

#define NUM_BOT_TYPES 7
#define NUM_MOTORS 4

void setup() {
  Serial.begin(115200);
  Serial.println(F("Writing Bot Type\n"));

  //! If you want to use a predefined robot from BotTypes.h, declare the index here:
  uint8_t index = 0; // also handles bot name index
  
  //! If you want to set custom bot and motor type, assign index appropriately, then assign these:
  eBOT_TYPE bot_type = lineman; 
  eMOTOR_TYPE motor_type = big;

  //* Write standard bot configuration from BotTypes.botConfigArray
  if (config.setConfig(index))
    Serial.println(F("Config write successful"));
  else
    Serial.println(F("Error writing bot config"));

  //* Write custom bot configuration
  // if(config.setConfig(index, bot_type, motor_type))
  //   Serial.println(F("Config write successful"));
  // else
  //   Serial.println(F("Error writing bot config"));  

  Serial.println();

  //* Read back for verification
  Serial.println(F("Readback:"));
  config.read(); // read the configuration from eeprom
  Serial.print(F(config.toString())); // print the configuration to the serial monitor

  Serial.println(F("Done"));
}

void loop() {}
