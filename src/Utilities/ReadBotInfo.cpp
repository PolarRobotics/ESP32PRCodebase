#include <Arduino.h>
// #include <Preferences.h>
// #include <PolarRobotics.h>
#include <Utilities/ConfigManager.h>

//! Follow (!) for instructions on how to read and utilize EEPROM bot config in other files

//! Instantiate ConfigManager
ConfigManager config;

void setup() {
  Serial.begin(115200);
  Serial.println(F("Reading:"));

  config.read(); //! In setup(), read config

  //! Use getBotType() and getMotorType() to retrieve the appropriate BotType and MotorType
  //! Then decide in your code (ex. main.cpp) how to handle this (ex. instantiating Robot subclass))
  // config.getBotType();
  // config.getMotorType();
  
  // Prints all properties of the config object retrieved from EEPROM, including bot and motor type
  // However this isn't accessible in code, so use the two methods above when not debugging
  Serial.print(F(config.toString()));

  Serial.println(F("Done"));
}

void loop() {}