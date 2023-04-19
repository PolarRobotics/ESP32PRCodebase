#include <Arduino.h>
#include <Preferences.h>
#include <PolarRobotics.h>
#include <Utilities/BotTypes.h>


Preferences preferences;
eBOT_TYPE bot_type;
eMOTOR_TYPE motor_type;

void setup() {
  Serial.begin(115200);
  Serial.println(F("Writing Bot Type"));

  // set the bot name you would like to assign to the bot
  uint8_t bot_name_index = 0;
  // set the bot type you would like to assign to the bot
  bot_type = quarterback;
  // set the motor type you would like to assign to the bot
  motor_type = mecanummotor;


  // open the "bot_config" namespace and set it to read/write
  preferences.begin("bot_config", false); 
  // store the bot name to preferences
  preferences.putUChar("bot_name_idx", bot_name_index);

  // store the bot type to preferences
  preferences.putUChar("bot_type", static_cast<uint8_t>(bot_type));
  
  // store the bot type to preferences
  preferences.putUChar("motor_type", static_cast<uint8_t>(motor_type));

  // close the namespace
  preferences.end();

  Serial.println("Done Writing... \nReading...");

  preferences.begin("bot_config", true);
  // read the bot name
  Serial.print(F("\r\nBot Name: "));
  Serial.print(preferences.getUChar("bot_name_idx"));
  Serial.print(F("\r\n"));

  // read the bot type
  Serial.print(F("Bot Type: "));
  Serial.print(preferences.getUChar("bot_type"));
  Serial.print(F("\r\n"));

  // read the bot type
  Serial.print(F("motor Type: "));
  Serial.print(preferences.getUChar("motor_type"));
  Serial.print(F("\r\n"));

  preferences.end();
}

void loop() {


}
