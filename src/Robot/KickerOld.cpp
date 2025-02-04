#include "KickerOld.h"

KickerOld::KickerOld(uint8_t kickerPin) {
  enabled = false;
  this->kickerPin = kickerPin;
  windupMotor.setup(kickerPin);
}

void KickerOld::action() {
  // Control the motor on the kicker
  if (ps5.Triangle())
    turnForward();
  else if (ps5.Cross())
    turnReverse();
  else
    stop();
}

void KickerOld::enable() {
  enabled = true;
}

void KickerOld::test() {
  if (enabled) {
    windupMotor.write(-1); //clockwise
    delay(3000);
    windupMotor.write(0); //stop
    delay(1000);
    windupMotor.write(1); //counter-clockwise
    delay(3000);
    windupMotor.write(0); //stop
  }
}

void KickerOld::turnForward() {
  if (enabled) {
    windupMotor.write(-0.5);
  }
}

void KickerOld::turnReverse() {
  if (enabled) {
    windupMotor.write(0.5);
  }
}

void KickerOld::stop() {
  if (enabled) {
    windupMotor.write(0);
  }
}