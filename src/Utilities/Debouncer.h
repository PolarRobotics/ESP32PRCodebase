#ifndef DEBOUNCER_H
#define DEBOUNCER_H

#include <PolarRobotics.h>

enum DebouncerState {
  base, active
};

class Debouncer {
  private:
    uint8_t lastUnstableState; // inputState from last iteration (may change sporadically)
    uint8_t lastStableState; // last debounced state (output)
    uint8_t lastLastStableState; // value of lastStableState from the last call to debounce(), used for wasToggled()
    unsigned long lastToggleTime; // the last time the input was toggled
    unsigned long debounceDelay; // the time an output must be stable before it is considered valid and is output

    uint8_t BASE_STATE;
    uint8_t ACTIVE_STATE;
  public:
    // construct debouncer with parameterized delay
    // recommend using pointer to debouncer object
    Debouncer(unsigned long delay, bool activeLow = false);

    // use to obtain "current" debounced state of button
    uint8_t debounce(uint8_t newState); 

    // use to execute an action *once* when the button is toggled *after debouncing*
    // it is assumed that debounce() is being called regularly
    uint8_t wasToggled();

    // use to execute an action *once* when the button is toggled *after debouncing*
    // calls both debounce() and wasToggled() consecutively to avoid potential misses
    uint8_t debounceAndToggled(uint8_t inputState);

    // use to execute an action when switched to a specific state *after debouncing*
    uint8_t wasSwitchedToState(DebouncerState state);

    // use to execute an action when switched to a specific state *after debouncing*
    // calls both debounce() and wasSwitchedToState() consecutively to avoid potential misses
    uint8_t debounceAndSwitchedTo(uint8_t inputState, DebouncerState targetState);

    // shorthand for debounceAndSwitchesTo(inputState, active)
    uint8_t debounceAndPressed(uint8_t inputState);
};

#endif // DEBOUNCER_H