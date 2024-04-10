//! How to instantiate: Lights& lights = Lights::getInstance();

#ifndef LIGHTS_H
#define LIGHTS_H
#include <FastLED.h>
#include <PolarRobotics.h>

#define NUM_LEDS 100
#define TIME_BETWEEN_TOGGLES 500
#define LIGHTS_SWITCH_TIME 1000
#define LIGHTS_UPDATE_TIME 20 // time between updates of the leds, in milliseconds

class Lights {
private:
  unsigned long lastToggleTime;
  uint8_t currState;
  CRGBArray<NUM_LEDS> leds;
  uint8_t iteration;
  uint8_t nextHomeState;
  uint8_t currentHomeState;
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
  void printDebugInfo();

  // LED Variables
  unsigned long tackleTime;
  unsigned long updateTime;
  const int switchTime = LIGHTS_SWITCH_TIME; // the time alotted to stay in the tackled state
  const int updateSwitchTime = LIGHTS_UPDATE_TIME; // debounce time for updating the leds when in the disco state
};

#endif // LIGHTS_H
