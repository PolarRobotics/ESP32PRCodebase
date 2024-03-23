#include "Robot/Lights.h"

// Function Definitions

Lights::Lights() {
  currState = PAIRING;
  homeState = AWAY;
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
      leds = CRGB::Green;
      break;
    }
    case DEFENSE: {
      leds = CRGB::White;
      break;
    }
    case TACKLED: {
      leds = CRGB::Red;
      break;
    }
    case DISCO: {
      for(int i = 0; i < NUM_LEDS; i++) {
        leds.fill_rainbow(iteration);
      }
      iteration++;
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
    switch (homeState) {
      case HOME:
        setLEDStatus(OFFENSE);
        homeState = AWAY;
        break;
      case AWAY:
        setLEDStatus(DEFENSE);
        homeState = LINEMAN;
        break;
      case LINEMAN:
        setLEDStatus(OFF);
        homeState = HOME;
        break;
    }
    lastToggleTime = millis();
  }
}

int Lights::returnStatus() {
  return static_cast<int>(this->currState);
}