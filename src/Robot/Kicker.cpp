#include "Kicker.h"

Kicker::Kicker(uint8_t kickerPin) {
  enabled = false;
  this->kickerPin = kickerPin;
  windupMotor.attach(kickerPin);
}

void Kicker::action() {
  // Control the motor on the kicker
  if (ps5.Triangle())
      turnForward();
  else if (ps5.Cross())
      turnReverse();
  else
      stop();
}

void Kicker::enable() {
  enabled = true;
      // TODO: move kicker pin arg, assignment, and attachment to constructor
      // TODO: rename setup() to enable(), have it only change `enabled`, and call it in main *after* pairing completes if the bot type is kicker
}

void Kicker::test() {
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

void Kicker::turnForward() {
  if (enabled)
    windupMotor.write(-0.5);
}

void Kicker::turnReverse() {
  if (enabled)
    windupMotor.write(0.5);
}

void Kicker::stop() {
  if (enabled)
    windupMotor.write(0);
}