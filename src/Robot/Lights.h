//! How to instantiate: Lights& lights = Lights::getInstance();

#ifndef LIGHTS_H
#define LIGHTS_H
#include <FastLED.h>
#include <PolarRobotics.h>

#define NUM_LEDS 100
#define TIME_BETWEEN_TOGGLES 500

class Lights {
private:
  unsigned long lastToggleTime;
  uint8_t currState;
  CRGBArray<NUM_LEDS> leds;
  uint8_t iteration;
  uint8_t nextState;
  uint8_t homeState;
  Lights();
public:
  // MUHAMMED ENUM PRAISE BE UPON HIM
  enum LEDState {
    PAIRING,     // yellow
    PAIRED,      // green then fade out
    UNPAIRED,
    HOME,        // green
    AWAY,        // white
    TACKLED,     // turn red when tackled
    DISCO,       // go crazy
    OFF
  };
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
  int homeStatus();
  void pairState(bool state);

  // LED Variables
  unsigned long tackleTime = 0;
  const int switchTime = 1000; //! KEEP THIS HERE!!!
};

#endif // LIGHTS_H
