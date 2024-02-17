#include "QuarterbackTurret.h"

QuarterbackTurret::QuarterbackTurret(
  uint8_t assemblyPin,
  uint8_t cradlePin,
  uint8_t turretPin,
  uint8_t flywheelLeftPin,
  uint8_t flywheelRightPin,
  uint8_t turretEncoderPinA,
  uint8_t turretEncoderPinB,
  uint8_t turretLaserPin
) {
  
  // set all state variables to default values,
  // except currentAssemblyAngle, currentCradleState, and currentRelativeHeading;
  // the positions of these mechanisms are initially unknown and assigned upon reset/homing

  this->enabled = false; // initially disable robot for safety
  this->initialized = false;
  this->runningMacro = false;
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

  this->targetRelativeHeading = 0; // while the initial heading is unknown, we want the heading to be zero

  this->stickFlywheel = 0;
  this->stickTurret = 0;

  // initiate motor objects
  // TODO: initiate assembly/tilter stepper motor with lib
  cradleActuator.setup(cradlePin, big_ampflow); // TODO: change to MotorInterface when merged
  turretMotor.setup(turretPin, falcon); // TODO: add encoder
  flywheelLeftMotor.setup(flywheelLeftPin, falcon);
  flywheelRightMotor.setup(flywheelRightPin, falcon);
}

void QuarterbackTurret::action() {
  //! Control Schema
  //* Touchpad: Emergency Stop
  if (ps5.Touchpad()) {
    emergencyStop();
  }
  //* Square: Toggle Flywheels/Turret On/Off (Safety Switch)
  else if (ps5.Square()) {
    if (!enabled) {
      setEnabled(true);
    } else {
      setEnabled(false);
    }
  }
  else if (!runningMacro) {
    //* Circle: Startup and Home (Reset or Zero Turret)
    if (ps5.Circle()) {
      // TODO: add a hold condition to this (hold for 1 sec to reset or something)
      if (!initialized) {
        reset();
      } else {
        zeroTurret();
      }
    }
    //* Triangle: Macro 1 - load from center
    else if (ps5.Triangle()) {
      loadFromCenter();
    }
    //* Cross: Macro 2 - handoff to runningback
    else if (ps5.Cross()) {
      handoff();
    } 
    //* Manual and Automatic Controls
    else {
      //* Right Trigger (R2): Fire (cradle/grabber forward)
      if (ps5.R2()) {
        moveCradle(forward);
      } else {
        moveCradle(back);
      }

      //* Options (Button): Switch Mode (toggle between auto/manual targeting)
      if (ps5.Options()) {
        switchMode();
      }
      //* Left Button (L1): Switch Target to Receiver 1
      else if (ps5.L1()) {
        switchTarget(receiver_1);
      }
      //* Right Button (R1): Switch Target to Receiver 2
      else if (ps5.R1()) {
        switchTarget(receiver_2);
      }
      //* Auto Mode
      else if (mode == automatic) {
        // TODO: Implement auto mode
        // do something based on current value of 'targetReceiver'
      }
      //* Manual Controls
      else {
        stickFlywheel = (ps5.LStickY() / 127.5f);
        stickTurret = (ps5.RStickX() / 127.5f);  

        // TODO: Implement proper heading-based turret control
        //* Right Stick X: Turret Control
        // Left = CCW, Right = CW
        if (fabs(stickTurret) > STICK_DEADZONE) {
          setTurretSpeed(stickTurret);
        } else {
          setTurretSpeed(0);
        }

        //* Left Stick Y: Flywheel Override
        if (fabs(stickFlywheel) > STICK_DEADZONE) {
          setFlywheelSpeed(stickFlywheel);
        } else {
          //* D-Pad Up: Increase flywheel speed by one stage
          if (ps5.Up()) {
            // todo: debounce
            adjustFlywheelSpeedStage(INCREASE);
          } 
          //* D-Pad Down: Decrease flywheel speed by one stage
          else if (ps5.Down()) {
            // todo: debounce
            adjustFlywheelSpeedStage(DECREASE);
          }
          else {
            setFlywheelSpeedStage(currentFlywheelStage);
          }
        }
      }
    }
  }

  // printDebug();
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

void QuarterbackTurret::moveTurret(int heading, bool relativeToRobot) {
  // todo
  if (relativeToRobot) {
    // todo: use encoder + laser to determine position
  } else { // relative to field
    // todo: use magnetometer
  }
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
  // update the motors so they are spinning at the new speed
  if (enabled) {
    // if current speed is not the passed speed, change the motor speed. this is only to avoid unnecessary writes
    if (fabs(currentFlywheelSpeed - absoluteSpeed) > STICK_DEADZONE) { 
      targetFlywheelSpeed = constrain(absoluteSpeed, -1.0, 1.0);
      flywheelLeftMotor.write(targetFlywheelSpeed);
      flywheelRightMotor.write(-targetFlywheelSpeed);
      currentFlywheelSpeed = targetFlywheelSpeed; //! for now, will probably need to change later, like an interrupt
    }
  } else {
    flywheelLeftMotor.write(0);
    flywheelRightMotor.write(0);
  }
}

void QuarterbackTurret::setFlywheelSpeedStage(FlywheelSpeed stage) {
  targetFlywheelStage = stage;
  setFlywheelSpeed(flywheelSpeeds[static_cast<uint8_t>(targetFlywheelStage)]);
  currentFlywheelStage = targetFlywheelStage;
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

void QuarterbackTurret::switchMode() {
  if (mode == manual) {
    switchMode(automatic);
  } else if (mode == automatic) {
    switchMode(manual);
  }
}

void QuarterbackTurret::switchMode(TurretMode mode) {
  this->mode = mode;
}

void QuarterbackTurret::switchTarget(TargetReceiver target) {
  switchMode(automatic);
  this->target = target;
  // todo: not sure if this needs more functionality?
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
  //TODO actually home to zero
}

void QuarterbackTurret::reset() {
  this->enabled = true;
  aimAssembly(straight);
  setFlywheelSpeedStage(slow_inwards);
  zeroTurret();
}

void QuarterbackTurret::printDebug() {
  Serial.print(F("enabled: "));
  Serial.print(enabled);
  Serial.print(F(" | stickTurret: "));
  Serial.print(stickTurret);
  Serial.print(F(" | stickFlywheel: "));
  Serial.print(stickFlywheel);
  Serial.print(" | currentTurretSpeed: ");
  Serial.println(currentTurretSpeed);
}