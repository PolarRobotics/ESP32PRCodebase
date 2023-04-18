// void updateLEDS(BOT_STATE status); //private
// void setRobotState(BOT_STATE state);
#pragma once
#include <Arduino.h>
#include <FastLED.h>
#include <PolarRobotics.h>

#define NUM_LEDS 100
#define TIME_BETWEEN_TOGGLES 500

// LED Variables
bool tackled = false;
static unsigned long tackleTime = 0;
static const int switchTime = 1000; // KEEP THIS HERE!!!
static unsigned long CURRENTTIME;
static int ledStatus = 0;

class Lights {
private:
    unsigned long lastToggleTime;
    uint8_t currState; // LEDState currState;
    CRGBArray<NUM_LEDS> leds;
    uint8_t iteration;
    bool m_isOffense;
    // int i, updateCount;
public:
    // MUHAMMED ENUM PRAISE BE UPON HIM
    enum LEDState {
        PAIRING,
        PAIRED,
        NOTPAIRED,
        TACKLE1,
        TACKLE2,
        OFF
    };
    Lights();
    void setupLEDS();
    void setLEDStatus(LEDState status);
    // void setLEDColor(uint8_t r, uint8_t g, )
    void updateLEDS();
    //   void runLoop(int count);
    void togglePosition();
    int returnStatus();
    void pairState(bool state);
};

// Function Definitions

Lights::Lights() {
    currState = PAIRING;
    m_isOffense = false;
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
      case NOTPAIRED: {
        for(int i = 0; i < NUM_LEDS; i ++){
            if(i % 2 == 0){leds[i] = CRGB::Black;}
            else{leds[i] = CRGB::White;}
          }
          break;
      }
      case TACKLE1: {
        for(int i = 0; i < NUM_LEDS; i ++){
            if(i % 2 == 0){leds[i] = CRGB::Red;}
            else{leds[i] = CRGB::White;}
          }
          break;
      }
      case TACKLE2: {
        for(int i = 0; i < NUM_LEDS; i ++){
            if(i % 2 == 0){leds[i] = CRGB::White;}
            else{leds[i] = CRGB::Red;}
          }
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
    // i think the ps5 library already does this we probably should check
    if (millis() - lastToggleTime >= TIME_BETWEEN_TOGGLES) {
        if (m_isOffense) {
            setLEDStatus(OFFENSE);
        }
        else {
            setLEDStatus(DEFENSE);
        }
        m_isOffense = !m_isOffense;
        lastToggleTime = millis();
    }
}

int Lights::returnStatus() {
    int status = 0;
    status = currState;
    return status;
}