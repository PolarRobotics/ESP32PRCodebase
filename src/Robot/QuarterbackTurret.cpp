#include "QuarterbackTurret.h"

QuarterbackTurret::QuarterbackTurret(
  uint8_t flywheelPin,
  uint8_t turretPin,
  uint8_t cradlePin
) {
  
  // set all state variables to default values,
  // except currentAssemblyAngle, currentCradleState, and currentTurretHeading;
  // the positions of these mechanisms are initially unknown and assigned upon reset/homing

  this->enabled = false; // initially disable robot for safety
  this->targetAssemblyAngle = straight; // while the initial state is unknown, we want it to be straight
  this->assemblyMoving = false; // it is safe to assume the assembly is not moving

  this->targetCradleState = back;
  this->cradleMoving = false;

  this->mode = manual; // start in manual mode by default, auto mode can be enabled by selecting a target
  this->target = receiver_1; // the default target is receiver 1, but this has no effect until the mode is switched to automatic

  this->currentFlywheelStage = stopped; // it is safe to assume the flywheels are stopped
  this->targetFlywheelStage = stopped;

  this->currentFlywheelSpeed = 0; // it is safe to assume the flywheels are stopped
  this->targetFlywheelSpeed = 0;

  this->flywheelManualOverride = false; // until/unless the left stick is active, this is false

  this->currentTurretSpeed = 0; // it is safe to assume the turret is stopped
  this->targetTurretSpeed = 0;

  this->targetTurretHeading = 0; // while the initial heading is unknown, we want the heading to be zero

}

void QuarterbackTurret::action() {
  // todo
}

void QuarterbackTurret::setTurretSpeed(float absoluteSpeed) {
  if (enabled) {
    targetTurretSpeed = constrain(absoluteSpeed, -1.0, 1.0);
    turretMotor.write(targetTurretSpeed);
    currentTurretSpeed = targetTurretSpeed; //! for now, will probably need to change later, like an interrupt
  } else {
    turretMotor.write(0);
  }
}

void QuarterbackTurret::moveTurret(int heading) {
  // todo
}

void QuarterbackTurret::aimAssembly(AssemblyAngle angle) {
  // todo 
  if (enabled) {
    // go to angle
  } else {
    // write 0
  }
}

void QuarterbackTurret::moveCradle(CradleState state) {
  // todo
  if (enabled) {
    targetCradleState = state;
    if (targetCradleState != currentCradleState) {
      if (targetCradleState == forward) {
        // move appropriate direction (forwards?)
        cradleActuator.write(1.0);
      } else if (targetCradleState == back) {
        // move other direction
        cradleActuator.write(-1.0);
      }
      currentCradleState = targetCradleState; //! for now, will probably need to change later, like an interrupt
    } else {
      cradleActuator.write(0);
    }
  } else {
    cradleActuator.write(0);
  }
}

void QuarterbackTurret::setFlywheelSpeed(double absoluteSpeed) {
  // Update the motors if they are spinning for the new speed
  if (enabled) {
    targetFlywheelSpeed = constrain(absoluteSpeed, -1.0, 1.0);
    flywheelMotor.write(targetFlywheelSpeed);
    currentFlywheelSpeed = targetFlywheelSpeed; //! for now, will probably need to change later, like an interrupt
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

void QuarterbackTurret::switchMode(TurretMode mode) {
  // todo
}

void QuarterbackTurret::switchTarget(TargetReceiver target) {
  // todo
}

void QuarterbackTurret::loadFromCenter() {
  // todo
}

void QuarterbackTurret::handoff() {
  // todo
}


void QuarterbackTurret::setEnabled(bool enabled) {
  this->enabled = enabled;
}

void QuarterbackTurret::emergencyStop() {
  this->enabled = false;
  setFlywheelSpeed(0); // this will not change the state variables since the bot is disabled
  setTurretSpeed(0);
}

void QuarterbackTurret::zeroTurret() {
  this->enabled = true;

  //TODO actually home to zero
}

void QuarterbackTurret::reset() {
  this->enabled = true;
  aimAssembly(straight);
  setFlywheelSpeedStage(slow_inwards);
  zeroTurret();
}