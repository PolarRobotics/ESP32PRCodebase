#include "Debouncer.h"

// based on: https://arduinogetstarted.com/tutorials/arduino-button-debounce

// Input: debounce delay (milliseconds)
Debouncer::Debouncer(unsigned long delay, bool activeLow) {
  if (activeLow) {
    BASE_STATE = HIGH;
    ACTIVE_STATE = LOW;
  } else {
    BASE_STATE = LOW;
    ACTIVE_STATE = HIGH;
  }

  // Initialize Variables
  this->lastLastStableState = BASE_STATE;
  this->lastStableState = BASE_STATE;
  this->lastUnstableState = BASE_STATE;
  this->lastToggleTime = 0;
  this->debounceDelay = delay;

}

// takes only input of 0 or 1, and outputs 0 or 1
// @param inputState: "current" call to debounce 
uint8_t Debouncer::debounce(uint8_t inputState) {
  // Serial.print(F("start: l_stab:"));
  // Serial.print(lastStableState);
  // Serial.print(F(", l_unst: "));
  // Serial.print(lastUnstableState);
  // Serial.print(F(", input: "));
  // Serial.print(inputState);

  // if the switch was toggled, update the last toggle time
  if (inputState != lastUnstableState) {
    lastToggleTime = millis();
    lastUnstableState = inputState;
  }

  // Serial.print(F(" | over delay?: "));
  // Serial.print((millis() - lastToggleTime) > debounceDelay);

  lastLastStableState = lastStableState;

  // test if the delay has been exceeded
  if ((millis() - lastToggleTime) > debounceDelay) {

    // Serial.print(F(" | stab_st changed?: "));
    // Serial.print(lastStableState != inputState);

    // if the state has changed, update it
    if (lastStableState != inputState) {
      lastStableState = inputState;

      // Serial.print(F(" | inputState == ACTIVE_STATE: "));
      // Serial.print(inputState == ACTIVE_STATE);

    }
  }

  // Serial.print(F(" | end: l_stab:"));
  // Serial.print(lastStableState);
  // Serial.print(F(", l_unst: "));
  // Serial.print(lastUnstableState);
  // Serial.print(F(", input: "));
  // Serial.println(inputState);

  return lastStableState;
}

uint8_t Debouncer::wasToggled() {
  return (lastLastStableState != lastStableState);
}

uint8_t Debouncer::debounceAndToggled(uint8_t inputState) {
  this->debounce(inputState);
  return this->wasToggled();
}

uint8_t Debouncer::wasSwitchedToState(DebouncerState state) {
  if (this->wasToggled()) {
    if (state == active) {
      return lastStableState == ACTIVE_STATE;
    } else { // state == base
      return lastStableState == BASE_STATE;
    }
  }
  return false;
}

uint8_t Debouncer::debounceAndSwitchedTo(uint8_t inputState, DebouncerState targetState) {
  this->debounce(inputState);
  return this->wasSwitchedToState(targetState);
}

uint8_t Debouncer::debounceAndPressed(uint8_t inputState) {
  return this->debounceAndSwitchedTo(inputState, active);
}