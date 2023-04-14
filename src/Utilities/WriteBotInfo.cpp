#include <Arduino.h>
#include <Preferences.h>
#include <PolarRobotics.h>

Preferences preferences;
eBOT_TYPE type;

void setup() {
  Serial.begin(115200);
  Serial.println(F("Writing Bot Type"));

  // set the bot type you would like to assign to the bot
  type = lineman;
  // set the motor type you would like to assign to the bot

  // open the "bot_type" namespace and set it to read/write
  preferences.begin("bot_type", false); 
  // store the bot type to preferences
  preferences.putUChar("bot_type", static_cast<uint8_t>(type));
  // read the bot type
  Serial.print(F("Bot Type: "));
  Serial.print(botType2String(preferences.getUChar("bot_type")));
  Serial.print(F("\r\n"));
  // close the namespace
  preferences.end();

  // open the "bot_type" namespace and set it to read/write
  preferences.begin("motor_type", false); 
  // store the bot type to preferences
  preferences.putUChar("motor_type", static_cast<uint8_t>(type));
  // read the bot type
  Serial.print(F("motor Type: "));
  Serial.print(motorType2String(preferences.getUChar("bot_type")));
  Serial.print(F("\r\n"));
  // close the namespace
  preferences.end();



  Serial.println("Done");

}

void loop() {


}
