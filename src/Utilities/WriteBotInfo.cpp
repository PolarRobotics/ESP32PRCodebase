#include <Arduino.h>

// #include <PolarRobotics.h>
#include <Utilities/ConfigManager.h>

// Preferences preferences;
ConfigManager config;

bool done = false;

void setup() {
  Serial.begin(115200);
  Serial.println(F("Writing Bot Type\n"));

  //! If you want to use a predefined robot from BotTypes.h, declare the index here:
  // based on https://docs.google.com/spreadsheets/d/1DswoEAcry9L9t_4ouKL3mXFgDMey4KkjEPFXULQxMEQ/edit#gid=0
  uint8_t index = 0; // also handles bot name index
  
  //! If you want to set custom bot and motor type, assign index appropriately, then assign these:
  BotType bot_type = lineman; 
  MotorType motor_type = small_12v;
  float gear_ratio = 1;
  float wheel_base = 10;

  //* Write standard bot configuration from BotTypes.botConfigArray
  // if (config.setConfig(index))
  //   Serial.println(F("Config write successful"));
  // else
  //   Serial.println(F("Error writing bot config"));

  //* Write custom bot configuration
  // if(config.setConfig(index, bot_type, motor_type, gear_ratio, wheel_base))
  //   Serial.println(F("Config write successful"));
  // else
  //   Serial.println(F("Error writing bot config"));  

  //* Read back for verification
  Serial.println(F("Readback:"));
  config.read(); // read the configuration from eeprom
  Serial.print(F(config.toString())); // print the configuration to the serial monitor

  Serial.println(F("Done"));
}

void loop() {}
