#include <Arduino.h>
#include "builtinLED.h"

/**
 * @brief Implementation of builtInLED.h
 * @authors Max Phillips
 * 
 * Provides a framework for accessing the LED built into
 * the ESP32 board. Is globally scoped to avoid multiple
 * drivers of the LED. Basically functions like a mutex.
 * 
 * This file implements the functions prototyped in 
 * `builtInLED.h`.
 * 
 **/

// Internal Variable for LED state
bool built_in_led_on = false;

/// @brief Tests if the built-in LED is on.
/// @return true if the LED is on, false if it is off.
bool builtInLedOn() {
  return built_in_led_on;
}

/// @brief Toggles the state of the built-in LED.
void toggleBuiltInLED() {
  built_in_led_on = !built_in_led_on;
  if (built_in_led_on) digitalWrite(LED_BUILTIN, HIGH);
  else digitalWrite(LED_BUILTIN, LOW);
}

/// @brief Sets the state of the built-in LED.
/// @param on true if it should be turned on, false if off. default is true.
void setBuiltInLED(bool on) {
  if (on) {
    built_in_led_on = true;
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    built_in_led_on = false;
    digitalWrite(LED_BUILTIN, LOW);
  }
}