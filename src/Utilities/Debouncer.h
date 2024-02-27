#ifndef DEBOUNCER_H
#define DEBOUNCER_H

#include <PolarRobotics.h>

class Debouncer {
  private:
    uint8_t lastUnstableState; // inputState from last iteration (may change sporadically)
    uint8_t lastStableState; // last debounced state (output)
    unsigned long lastToggleTime; // the last time the input was toggled
    unsigned long debounceDelay; // the time an output must be stable before it is considered valid and is output

    uint8_t BASE_STATE;
    uint8_t ACTIVE_STATE;
  public:
    Debouncer(unsigned long delay, bool activeLow = false);
    uint8_t debounce(uint8_t newState);
};

#endif // DEBOUNCER_H