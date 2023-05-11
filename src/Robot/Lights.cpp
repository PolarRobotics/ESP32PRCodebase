#include "Robot/Lights.h"

// Function Definitions

Lights::Lights() {
  currState = PAIRING;
  isOffense = false;
}

void Lights::setupLEDS() {
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  // FastLED.setMaxPowerInVoltsAndMilliamps(5, 500); // Power Failsafe
  // Clears LEDs when code is updated
  FastLED.clear();

  updateLEDS();
  FastLED.setBrightness(110);
}

// To set LED status
void Lights::setLEDStatus(LEDState status) {
  this->currState = status;
  updateLEDS();
}

// To change LED color
void Lights::updateLEDS() {
  switch (this->currState) {
    case PAIRING: {
      leds = CRGB::DarkOrange;
      break;
    }
    case PAIRED: {
      leds = CRGB::Purple;
      break;
    }
    case UNPAIRED: {
      for (int i = 0; i < NUM_LEDS; i ++){
        if (i % 2 == 0) { leds[i] = CRGB::Black; }
        else { leds[i] = CRGB::White; }
      }
      break;
    }
    case OFFENSE: {
      for(int i = 0; i < NUM_LEDS; i ++){
        if (i % 2 == 0) { leds[i] = CRGB::Blue; }
        else { leds[i] = CRGB::Green; }
      }
      break;
    }
    case DEFENSE: {
      leds = CRGB::Green;
      break;
    }
    case TACKLED: {
      leds = CRGB::Red;
      break;
    }
    case OFF: {
      leds = CRGB::Black;
      break;
    }
  }
  FastLED.show();
}


void Lights::togglePosition() {
  // debounce makes sure you cant hold down the button, 
  // I think the ps5 library already does this, but we probably should check
  if (millis() - lastToggleTime >= TIME_BETWEEN_TOGGLES) {
    if (isOffense) {
      setLEDStatus(OFFENSE);
    }
    else {
      setLEDStatus(DEFENSE);
    }
    isOffense = !isOffense;
    lastToggleTime = millis();
  }
}

int Lights::returnStatus() {
  // TODO: should be able to refactor this to `return (int) status` or failing that `static_cast<int>(status)`
  int status = 0;
  status = currState;
  return status;
}