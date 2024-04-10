#include "Robot/Lights.h"

// Function Definitions

Lights::Lights() {
  currState = PAIRING;
  nextHomeState = AWAY;
  currentHomeState = AWAY;

  tackleTime = millis();
  updateTime = millis();
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
    case HOME: {
      leds = CRGB::Green;
      break;
    }
    case AWAY: {
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
    switch (nextHomeState) {
      case HOME:
        setLEDStatus(HOME);
        currentHomeState = HOME;
        nextHomeState = AWAY;
        break;
      case AWAY:
        setLEDStatus(AWAY);
        currentHomeState = AWAY;
        nextHomeState = OFF;
        break;
      case OFF:
        setLEDStatus(OFF);
        currentHomeState = OFF;
        nextHomeState = HOME;
        break;
    }
    lastToggleTime = millis();
  }
}

int Lights::returnStatus() {
  return static_cast<int>(this->currState);
}

int Lights::homeStatus() {
  return static_cast<int>(this->currentHomeState);
}

void Lights::printDebugInfo() {
  Serial.print(F("Lights: currState: "));
  Serial.print(static_cast<int>(this->currState));
  Serial.print(F(" currentHomeState: "));
  Serial.print(static_cast<int>(this->currentHomeState));
  Serial.print(F("\n"));
}
