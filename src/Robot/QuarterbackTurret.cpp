#include "QuarterbackTurret.h"

void QuarterbackTurret::moveTurret(float power) {

}

void QuarterbackTurret::setFlywheelSpeed(double absoluteSpeed) {
  // Update the motors if they are spinning for the new speed
  if (enabled) {
    targetFlywheelSpeed = constrain(absoluteSpeed, -1.0, 1.0);
    flywheelMotor.write(targetFlywheelSpeed);
    currentFlywheelSpeed = targetFlywheelSpeed;
  } else {
    flywheelMotor.write(0);
  }
}

void QuarterbackTurret::setFlywheelSpeedStage(FlywheelSpeed stage) {
    setFlywheelSpeed(flywheelSpeeds[static_cast<uint8_t>(stage)]);
}

void QuarterbackTurret::adjustFlywheelSpeedStage(SpeedStatus speed) {
    uint8_t idx = static_cast<uint8_t>(currentFlywheelStage);
    
    // Change the speed stage based on whether the user wants to increase or decrease
    if (speed == INCREASE && idx < QB_TURRET_NUM_SPEEDS - 1) {
        idx++;
    } else if (speed == DECREASE && idx > 0) {
        idx--;
    }

    setFlywheelSpeedStage(static_cast<FlywheelSpeed>(idx));
}