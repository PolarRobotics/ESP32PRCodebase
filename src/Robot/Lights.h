//! How to instantiate: Lights& lights = Lights::getInstance();

#ifndef LIGHTS_H
#define LIGHTS_H
#include <FastLED.h>
#include <PolarRobotics.h>

// led strip:
#define NUM_LEDS 100
#define TIME_BETWEEN_TOGGLES 500
#define LIGHTS_SWITCH_TIME 1000
#define LIGHTS_UPDATE_TIME 20 // time between updates of the leds, in milliseconds

// receiver indicator array
#define NUM_INDICATOR_LEDS 9 // 3x3#define NUM_ROWS 

constexpr uint16_t CROSSPATTERN[NUM_INDICATOR_LEDS] {
  0x000000, 0xFFFFFF, 0x000000,  // black, white, black
  0xFFFFFF, 0xFFFFFF, 0xFFFFFF,  // white, white, white
  0x000000, 0xFFFFFF, 0x000000   // black, white, black
};

constexpr uint16_t YPATTERN[NUM_INDICATOR_LEDS] {
  0xFFFFFF, 0x000000, 0xFFFFFF,  // black, white, black
  0x000000, 0xFFFFFF, 0x000000,  // white, white, white
  0x000000, 0xFFFFFF, 0x000000   // black, white, black
};

class Lights {
private:
  // led strip parameters:
  unsigned long lastToggleTime;
  uint8_t currState;
  CRGBArray<NUM_LEDS> leds;
  uint8_t iteration;
  uint8_t nextHomeState;
  uint8_t currentHomeState;

  // receiver indicator board 
  uint8_t currPattern;
  CRGBArray<NUM_LEDS> indicatorBoard;
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
  enum Pattern {
    CROSS,
    Y
  };
  static Lights& getInstance() {
    static Lights instance;
    return instance;
  }
  Lights(const Lights& obj) = delete; // delete copy constructor
  void operator=(Lights const&)  = delete; // delete set operator
  void setupLEDS();

  void setupIndicator();
  void indicatorPattern(Pattern pattern);
  
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
