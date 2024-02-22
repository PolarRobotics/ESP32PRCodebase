#include <Arduino.h>

// #include <PolarRobotics.h>
#include <Utilities/ConfigManager.h>

// Preferences preferences;
ConfigManager config;

bool validConfig = false;

void setup() {
  Serial.begin(115200);
  Serial.println(F("Writing Bot Type\n"));

  //* STANDARD BOT CONFIGURATION
  //! If you want to use a predefined robot from BotTypes.h, declare the index here:
  // based on https://docs.google.com/spreadsheets/d/1DswoEAcry9L9t_4ouKL3mXFgDMey4KkjEPFXULQxMEQ/edit#gid=0
  //! Please reset to zero when you are done uploading to avoid merge conflicts.
  uint8_t index = BOT_QB_TURRET; // also handles bot name index

  //* CUSTOM BOT CONFIGURATION
  //! If you want to set custom bot and motor type, assign index appropriately, then assign these:
  BotType bot_type = lineman; 
  MotorType motor_type = small_12v;
  float gear_ratio = 1;
  float wheel_base = 10;
  float r_min = 9.00f;
  float r_max = 36.00f;

  //! Do not use custom config except for TEMPORARY testing, OR:
  //! DO NOT USE WITHOUT DOCUMENTING YOUR CHANGES BY:
  //! - sending a message in the programming chat in Discord, AND:
  //! - making a comment in the bot types spreadsheet (see link above), AND:
  //! - mirroring your changes into botConfigArray in BotTypes.h and pushing your changes to a new branch as appropriate
  constexpr bool useCustomConfig = false;

  if (useCustomConfig) {
    //* Write custom bot configuration
    validConfig = config.setConfig(index, bot_type, motor_type, gear_ratio, wheel_base, r_min, r_max);
  } else {
    //* Write standard bot configuration from BotTypes.botConfigArray
    validConfig = config.setConfig(index);
  }

  if (validConfig) {
    Serial.println(F("Config write successful"));
  } else {
    Serial.println(F("Error writing bot config"));
  }

  //* Read back for verification
  Serial.println(F("Readback:"));
  config.read(); // read the configuration from eeprom
  Serial.print(F(config.toString())); // print the configuration to the serial monitor

  Serial.println(F("Done"));
}

void loop() {}
