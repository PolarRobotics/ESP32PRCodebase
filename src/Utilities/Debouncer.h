#ifndef DEBOUNCER_H
#define DEBOUNCER_H

#include <PolarRobotics.h>

class Debouncer {
  private:
    uint8_t outputState;
    uint8_t currentState;
    uint8_t lastState;
    unsigned long lastToggleTime;
    unsigned long debounceDelay;

    uint8_t BASE_STATE;
    uint8_t ACTIVE_STATE;
  public:
    Debouncer(unsigned long delay, bool activeLow = false);
    uint8_t debounce(uint8_t newState);
};

#endif // DEBOUNCER_H