#include <Arduino.h>
#include "builtinLED.h"

bool built_in_led_on = false;

bool builtInLedOn() {
  return built_in_led_on;
}

void toggleBuiltInLED() {
  built_in_led_on = !built_in_led_on;
  if (built_in_led_on) digitalWrite(LED_BUILTIN, HIGH);
  else digitalWrite(LED_BUILTIN, LOW);
}