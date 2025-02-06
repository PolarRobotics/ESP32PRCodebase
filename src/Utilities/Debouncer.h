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
    // construct debouncer with parameterized delay (in milliseconds)
    // recommend using pointer to debouncer object
    // default is active high ('1' is active state). set second argument to 'true' to make active low ('0')
    Debouncer(unsigned long delay, bool activeLow = false);

    // must be called regularly (i.e., in a loop)
    // whatever you call to check your button/sensor, wrap it in this
    // takes only input of 0 or 1, and outputs 0 or 1
    /// @param inputState: "current" call to debounce 
    uint8_t debounce(uint8_t inputState); 

    // should be used to obtain "current" debounced state of button
    uint8_t isActive();

    // should be used to enable execution of an action *once* when the button is toggled *after debouncing*
    // in other words, it checks whether the button has changed debounced state
    // it is assumed that debounce() is being called regularly
    uint8_t wasToggled();

    // should be used to enable execution of an action *once* when the button is toggled *after debouncing*
    // calls both debounce() and wasToggled() consecutively to avoid potential misses
    uint8_t debounceAndToggled(uint8_t inputState);

    // should be used to enable execution of an action when switched to a specific state *after debouncing*
    // calls wasToggled() and checks it against the passed `state` parameter
    uint8_t wasSwitchedToState(DebouncerState state);

    // should be used to enable execution of an action when switched to a specific state *after debouncing*
    // calls both debounce() and wasSwitchedToState() consecutively to avoid potential misses
    // not as useful as debounceAndPressed() because usually you have no reason to switch based on the inactive state
    uint8_t debounceAndSwitchedTo(uint8_t inputState, DebouncerState targetState);

    //* usually you will use this function
    // should be used to enable execution of an action when a button has been pressed (after debouncing)
    //* it is assumed that the "active" state of the button corresponds to the "pressed" state
    // shorthand for debounceAndSwitchesTo(inputState, active)
    // calls both debounce() and wasSwitchedToState() automatically
    uint8_t debounceAndPressed(uint8_t inputState);
};

//* Example Usage
#if 0
// in header file
#define DB_EXAMPLE_DELAY 500L

// in header file/class
Debouncer* dbExample;

// in source file/constructor
dbExample = new Debouncer(DB_EXAMPLE_DELAY);

// in source file/loop
//* use case 1
if (dbExample->debounceAndPressed(ps5.Circle())) {
  // do something...
}

//* use case 2
uint8_t debouncedValue = dbExample->debounce(ps5.Circle()); // run repeatedly
// check debouncedValue, do something with it, maybe graph it, idk...

//* use case 3
// this is basically a binary trigger
// it will toggle on a change in debounced state
if (dbExample->debounceAndToggled(ps5.Circle())) {
  // do something
  // you could also do something different depending on the current state
  if (dbExample->isActive()) {
    // do something else
  }
}
#endif

#endif // DEBOUNCER_H