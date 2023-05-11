// void updateLEDS(BOT_STATE status); //private
// void setRobotState(BOT_STATE state);
#ifndef LIGHTS_H
#define LIGHTS_H
#include <FastLED.h>
#include <PolarRobotics.h>

#define NUM_LEDS 100
#define TIME_BETWEEN_TOGGLES 500

// TODO: rename `m_` variables
// TODO: refactor to singleton paradigm instead of using extern function for instance of LEDs https://refactoring.guru/design-patterns/singleton/cpp/example#example-0

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
      PAIRING,     // yellow
      PAIRED,      // green then fade out
      NOTPAIRED,
      OFFENSE,     // blue and green
      DEFENSE,     // green
      TACKLED,     // turn red when tackled
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

#endif // LIGHTS_H
