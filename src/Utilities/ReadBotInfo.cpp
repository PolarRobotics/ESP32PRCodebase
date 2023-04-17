#include <Arduino.h>
#include <Preferences.h>
#include <PolarRobotics.h>

Preferences preferences;

void setup() {
  Serial.begin(115200);
  Serial.println("Reading");

  // open the "bot_name_idx" namespace and set it to read/write
  preferences.begin("bot_name_idx", true); 
  // read the bot name
  Serial.print(F("Bot Name: "));
  Serial.print(preferences.getUChar("bot_name_idx"));
  Serial.print(F("\r\n"));
  // close the namespace
  preferences.end();

  // open the "bot_type" namespace and set it to read/write
  preferences.begin("bot_type", true); 
  // read the bot type
  Serial.print(F("Bot Type: "));
  Serial.print(preferences.getUChar("bot_type"));
  Serial.print(F("\r\n"));
  // close the namespace
  preferences.end();

  // open the "motor_type" namespace and set it to read/write
  preferences.begin("motor_type", true); 
  // read the bot type
  Serial.print(F("motor Type: "));
  Serial.print(preferences.getUChar("bot_type"));
  Serial.print(F("\r\n"));
  // close the namespace
  preferences.end();

  Serial.println("Done");
}

void loop() {

}
