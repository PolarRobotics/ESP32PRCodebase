//! How to instantiate: Lights& lights = Lights::getInstance();

#ifndef LIGHTS_H
#define LIGHTS_H
#include <FastLED.h>
#include <PolarRobotics.h>

#define NUM_LEDS 100
#define TIME_BETWEEN_TOGGLES 500

class Lights {
public:

  // MUHAMMED ENUM PRAISE BE UPON HIM
  enum LEDState {
    PAIRING,     // yellow
    PAIRED,      // green then fade out
    UNPAIRED,
    OFFENSE,     // blue and green
    DEFENSE,     // green
    TACKLED,     // turn red when tackled
    DISCO,       // go crazy
    OFF
  };
  enum HomeState {
    HOME,
    AWAY,
    LINEMAN
  };

private:

  unsigned long lastToggleTime;
  LEDState currState;
  CRGBArray<NUM_LEDS> leds;
  uint8_t iteration;
  HomeState homeState;
  Lights();\
  
public:

  static Lights& getInstance() {
    static Lights instance;
    return instance;
  }
  Lights(const Lights& obj) = delete; // delete copy constructor
  void operator=(Lights const&)  = delete; // delete set operator
  void setupLEDS();
  void setLEDStatus(LEDState status);
  void updateLEDS();
  void togglePosition();
  int returnStatus();
  void pairState(bool state);

  // LED Variables
  unsigned long tackleTime = 0;
  const int switchTime = 1000; //! KEEP THIS HERE!!!
};

#endif // LIGHTS_H
