#include "QuarterbackTurret.h"

// "define" static members to satisfy linker
uint8_t QuarterbackTurret::turretEncoderPinA;
uint8_t QuarterbackTurret::turretEncoderPinB;
uint8_t QuarterbackTurret::turretEncoderStateB;
int64_t QuarterbackTurret::turretEncoderCount;

void QuarterbackTurret::turretEncoderISR() {
  turretEncoderStateB = digitalRead(turretEncoderPinB);

  if (turretEncoderStateB == 1) {
    turretEncoderCount++;
  } else if (turretEncoderStateB == 0) {
    turretEncoderCount--;
  }
}

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

  this->currentCradleState = back;
  this->targetCradleState = back;
  this->cradleMoving = false;
  this->cradleStartTime = 0;

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

  // turret laser setup
  this->turretLaserPin = turretLaserPin;
  this->turretLaserState = 0;
  pinMode(turretLaserPin, INPUT_PULLUP); //! will be 1 when at home position or main power is off (the latter is electrically unavoidable)

  // encoder setup
  QuarterbackTurret::turretEncoderPinA = turretEncoderPinA;
  QuarterbackTurret::turretEncoderPinB = turretEncoderPinB;
  QuarterbackTurret::turretEncoderCount = 0;
  pinMode(turretEncoderPinA, INPUT_PULLUP);
  pinMode(turretEncoderPinB, INPUT);
  attachInterrupt(turretEncoderPinA, turretEncoderISR, RISING);

  // initiate motor objects
  // TODO: initiate assembly/tilter stepper motor with lib
  cradleActuator.setup(cradlePin, big_ampflow); // TODO: change to MotorInterface when merged
  turretMotor.setup(turretPin, falcon); // TODO: add encoder
  flywheelLeftMotor.setup(flywheelLeftPin, falcon);
  flywheelRightMotor.setup(flywheelRightPin, falcon);

  
  
  // initialize debouncers
  this->dbOptions = new Debouncer(QB_BASE_DEBOUNCE_DELAY);
  this->dbSquare = new Debouncer(QB_BASE_DEBOUNCE_DELAY);
  this->dbDpadUp = new Debouncer(QB_BASE_DEBOUNCE_DELAY);
  this->dbDpadDown = new Debouncer(QB_BASE_DEBOUNCE_DELAY);
  this->dbCircle = new Debouncer(750L);

}

void QuarterbackTurret::action() {
  //! Control Schema
  //* Touchpad: Emergency Stop
  if (ps5.Touchpad()) {
    emergencyStop();
  }
  //* Square: Toggle Flywheels/Turret On/Off (Safety Switch)
  else if (dbSquare->debounceAndPressed(ps5.Square())) {
    if (!enabled) {
      setEnabled(true);
    } else {
      setEnabled(false);
    }
  }
  //! Ignore non-emergency inputs if running a macro
  else if (!runningMacro) {
    //* Circle: Startup and Home (Reset or Zero Turret)
    if (dbCircle->debounceAndPressed(ps5.Circle())) {
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
      // Do not fire unless moving forward (do not fire when intaking or stopped)
      if (currentFlywheelSpeed > STICK_DEADZONE && ps5.R2()) {
        moveCradle(forward);
      } else {
        moveCradle(back);
      }

      //* Options (Button): Switch Mode (toggle between auto/manual targeting)
      if (dbOptions->debounceAndPressed(ps5.Options())) {
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
          setTurretSpeed(0.5 * stickTurret);
        } else {
          setTurretSpeed(0);
        }

        //* Left Stick Y: Flywheel Override
        if (fabs(stickFlywheel) > STICK_DEADZONE) {
          setFlywheelSpeed(stickFlywheel);
        } else {
          //* D-Pad Up: Increase flywheel speed by one stage
          if (dbDpadUp->debounceAndPressed(ps5.Up())) {
            Serial.println(F("increasing flywheel speed"));
            adjustFlywheelSpeedStage(INCREASE);
          } 
          //* D-Pad Down: Decrease flywheel speed by one stage
          else if (dbDpadDown->debounceAndPressed(ps5.Down())) {
            adjustFlywheelSpeedStage(DECREASE);
          }
          else {
            setFlywheelSpeedStage(currentFlywheelStage);
          }
        }
      }
    }
  }

  printDebug();
}

void QuarterbackTurret::setTurretSpeed(float absoluteSpeed) {
  if (enabled) {
    targetTurretSpeed = constrain(absoluteSpeed, -1.0, 1.0);
    turretMotor.write(-targetTurretSpeed); // flip direction so that + is CW and - is CCW
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
  if (enabled) {
    if (!cradleMoving) {
      targetCradleState = state;

      // Serial.print(F("current state: "));
      // if (currentCradleState == forward) {
      //   Serial.print(F("forward  | "));
      // } else if (currentCradleState == back) {
      //   Serial.print(F("backward  | "));
      // }

      // Serial.print(F("target state: "));
      // if (targetCradleState == forward) {
      //   Serial.print(F("forward  | "));
      // } else if (targetCradleState == back) {
      //   Serial.print(F("backward  | "));
      // }

      if (targetCradleState != currentCradleState) {
        // Serial.print(F("target neq current  | "));
        if (targetCradleState == forward) {
          // move forwards
          cradleActuator.write(1.0);
          cradleStartTime = millis();
          cradleMoving = true;
          // Serial.print(F("cradle moving forward  | "));
        } else if (targetCradleState == back) {
          // move backwards
          cradleActuator.write(-1.0);
          cradleStartTime = millis();
          cradleMoving = true;
          // Serial.print(F("cradle moving backward  | "));
        }
        
      }

      // Serial.print(F("past delay? "));
      // Serial.print((millis() - cradleStartTime) > QB_CRADLE_TRAVEL_DELAY);
      // Serial.print(F(" | "));

    } else if ((millis() - cradleStartTime) > QB_CRADLE_TRAVEL_DELAY) {
      currentCradleState = targetCradleState;
      cradleMoving = false;
      cradleActuator.write(0);
      // Serial.print(F("cradle stopped  | "));
    }
  } else {
    cradleActuator.write(0);
  }

  // Serial.println();
}

void QuarterbackTurret::setFlywheelSpeed(double absoluteSpeed) {
  // update the motors so they are spinning at the new speed
  if (enabled) {
    // if current speed is not the passed speed, change the motor speed. this is only to avoid unnecessary writes
    if (fabs(currentFlywheelSpeed - absoluteSpeed) > STICK_DEADZONE) {
      // constrain to the first and last values of the flywheel speed array.
      // the first value should be the slow intake speed -- the flywheels should NEVER spin more quickly *inwards* than this.
      // the last value should be the maximum speed (ordinarily 1, but we may change this).
      targetFlywheelSpeed = constrain(absoluteSpeed, flywheelSpeeds[0], flywheelSpeeds[QB_TURRET_NUM_SPEEDS - 1]);
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
  this->runningMacro = true;
  aimAssembly(straight);
  setFlywheelSpeedStage(slow_inwards);
  moveCradle(back);
  zeroTurret();
  this->runningMacro = false;
}

void QuarterbackTurret::handoff() {
  this->runningMacro = true;
  aimAssembly(straight);
  setFlywheelSpeedStage(slow_outwards);
  // TODO: Rotate 180
  moveCradle(forward);
  this->runningMacro = false;
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
  this->runningMacro = true;
  Serial.println(F("zero called"));
  // pin will only read high if the main power is on and the laser sensor is triggered
  while (digitalRead(turretLaserPin) == LOW) {
    Serial.print(F("zeroing, read = "));
    Serial.println(digitalRead(turretLaserPin));
    setTurretSpeed(QB_HOME_PCT);
  }
  setTurretSpeed(0);
  Serial.println(F("zeroed"));
  this->runningMacro = false;
}

void QuarterbackTurret::reset() {
  this->enabled = true;
  this->runningMacro = true;
  loadFromCenter();
  this->initialized = true;
  this->runningMacro = false;
}

void QuarterbackTurret::printDebug() {
  /*
  Serial.print(F("enabled: "));
  Serial.print(enabled);
  Serial.print(F(" | stickTurret: "));
  Serial.print(stickTurret);
  Serial.print(F(" | stickFlywheel: "));
  Serial.print(stickFlywheel);
  Serial.print(F(" | currentTurretSpeed: "));
  Serial.println(currentTurretSpeed);
  */
  // Serial.print(F("turretLaserState: "));
  // Serial.println(digitalRead(turretLaserPin));
  Serial.print(F("turretEncoderCount: "));
  Serial.println(turretEncoderCount);
}