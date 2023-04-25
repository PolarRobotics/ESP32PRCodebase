#include <Arduino.h>
#include <Preferences.h>
#include <PolarRobotics.h>
#include <Utilities/ConfigReader.h>

// Preferences preferences;
ConfigReader Config;

void setup() {
  Serial.begin(115200);
  Serial.println(F("Reading"));

  Config.read();
  
  Serial.print(F(Config.toString()));

  Serial.println(F("Done"));
}

void loop() {

}

  // open the "bot_name_idx" namespace and set it to read/write
  // preferences.begin("bot_config", true); 
  // // read the bot name
  // Serial.print(F("Bot Name: "));
  // Serial.print(preferences.getUChar("bot_name_idx"));
  // Serial.print(F("\r\n"));
 
  // // read the bot type
  // Serial.print(F("Bot Type: "));
  // Serial.print(preferences.getUChar("bot_type"));
  // Serial.print(F("\r\n"));

  // // read the bot type
  // Serial.print(F("motor Type: "));
  // Serial.print(preferences.getUChar("bot_type"));
  // Serial.print(F("\r\n"));
  // // close the namespace
  // preferences.end();