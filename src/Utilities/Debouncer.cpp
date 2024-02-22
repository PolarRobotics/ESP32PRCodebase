#include "Debouncer.h"

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
  this->currentState = BASE_STATE;
  this->lastState = BASE_STATE;
  this->lastToggleTime = 0;
  this->debounceDelay = delay;
  this->outputState = BASE_STATE;

}

// takes only input of 0 or 1, and outputs 0 or 1
uint8_t Debouncer::debounce(uint8_t newState) {
  Serial.print(F("start: last:"));
  Serial.print(lastState);
  Serial.print(F(", curr: "));
  Serial.print(currentState);
  Serial.print(F(", new: "));
  Serial.print(newState);
  Serial.print(F(", out: "));
  Serial.print(outputState);

  // if the switch was toggled, update the last toggle time
  if (newState != lastState) {
    lastToggleTime = millis();
  }

  Serial.print(F(" | over delay?: "));
  Serial.print((millis() - lastToggleTime) > debounceDelay);

  // test if the delay has been exceeded
  if ((millis() - lastToggleTime) > debounceDelay) {

    Serial.print(F(" | state changed?: "));
    Serial.print(newState != currentState);

    // if the state has changed, update it
    if (newState != currentState) {
      currentState = newState;

      Serial.print(F(" | currentState == ACTIVE_STATE: "));
      Serial.print(currentState == ACTIVE_STATE);

      if (currentState == ACTIVE_STATE) {
        outputState = !outputState;
      }
    }
  }

  // save new states
  lastState = currentState;
  currentState = newState;

  Serial.print(F(" | end: last:"));
  Serial.print(lastState);
  Serial.print(F(", curr: "));
  Serial.print(currentState);
  Serial.print(F(", new: "));
  Serial.print(newState);
  Serial.print(F(", out: "));
  Serial.println(outputState);

  return outputState;
}