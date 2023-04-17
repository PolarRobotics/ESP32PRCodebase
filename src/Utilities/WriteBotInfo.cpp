#include <Arduino.h>
#include <Preferences.h>
#include <PolarRobotics.h>

Preferences preferences;
eBOT_TYPE bot_type;
eMOTOR_TYPE motor_type;

void setup() {
  Serial.begin(115200);
  Serial.println(F("Writing Bot Type"));

  // set the bot name you would like to assign to the bot
  uint8_t bot_name_index = 0;
  // set the bot type you would like to assign to the bot
  bot_type = lineman;
  // set the motor type you would like to assign to the bot
  motor_type = small;


  // open the "bot_name_idx" namespace and set it to read/write
  preferences.begin("bot_name_idx", false); 
  // store the bot name to preferences
  preferences.putUChar("bot_name_idx", bot_name_index);
  // read the bot name
  Serial.print(F("Bot Name: "));
  Serial.print(preferences.getUChar("bot_name_idx"));
  Serial.print(F("\r\n"));
  // close the namespace
  preferences.end();

  // open the "bot_type" namespace and set it to read/write
  preferences.begin("bot_type", false); 
  // store the bot type to preferences
  preferences.putUChar("bot_type", static_cast<uint8_t>(bot_type));
  // read the bot type
  Serial.print(F("Bot Type: "));
  Serial.print(preferences.getUChar("bot_type"));
  Serial.print(F("\r\n"));
  // close the namespace
  preferences.end();

  // open the "motor_type" namespace and set it to read/write
  preferences.begin("motor_type", false); 
  // store the bot type to preferences
  preferences.putUChar("motor_type", static_cast<uint8_t>(motor_type));
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
