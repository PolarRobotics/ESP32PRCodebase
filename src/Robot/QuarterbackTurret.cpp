#include "QuarterbackTurret.h"

// "define" static members to satisfy linker
uint8_t QuarterbackTurret::turretEncoderPinA;
uint8_t QuarterbackTurret::turretEncoderPinB;
uint8_t QuarterbackTurret::turretEncoderStateB;
int32_t QuarterbackTurret::currentTurretEncoderCount;

void QuarterbackTurret::turretEncoderISR() {
  turretEncoderStateB = digitalRead(turretEncoderPinB);

  if (turretEncoderStateB == 1) {
    currentTurretEncoderCount++;
  } else if (turretEncoderStateB == 0) {
    currentTurretEncoderCount--;
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
  // except currentAssemblyAngle, currentRelativeHeading, currentRelativeTurretCount
  // the positions of these mechanisms are initially unknown and assigned upon reset/homing

  this->enabled = false; // initially disable robot for safety
  this->initialized = false;
  this->runningMacro = false;
  this->targetAssemblyAngle = straight; // while the initial state is unknown, we want it to be straight
  this->assemblyMoving = false; // it is safe to assume the assembly is not moving

  this->currentCradleState = forward; // in case the startup state is strange, force the cradle to move back once on startup
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
  this->targetTurretEncoderCount = 0;
  
  this->turretMoving = false;

  this->stickFlywheel = 0;
  this->stickTurret = 0;

  // turret laser setup
  this->turretLaserPin = turretLaserPin;
  this->turretLaserState = 0;
  pinMode(turretLaserPin, INPUT_PULLUP); //! will be 1 when at home position or main power is off (the latter is electrically unavoidable)

  // encoder setup
  QuarterbackTurret::turretEncoderPinA = turretEncoderPinA;
  QuarterbackTurret::turretEncoderPinB = turretEncoderPinB;
  QuarterbackTurret::currentTurretEncoderCount = 0;
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

  this->dbCircle = new Debouncer(QB_CIRCLE_HOLD_DELAY);
  this->dbTriangle = new Debouncer(QB_TRIANGLE_HOLD_DELAY);
  this->dbCross = new Debouncer(QB_CROSS_HOLD_DELAY);

  this->dbTurretInterpolator = new Debouncer(QB_TURRET_INTERPOLATION_DELAY);

}

void QuarterbackTurret::action() {
  //! Control Schema
  //* Touchpad: Emergency Stop
  //* Square: Toggle Flywheels/Turret On/Off (Safety Switch)
  // above two inputs are registered in `testForDisableOrStop()` since this is re-used in blocking routines
  //! Ignore non-emergency inputs if running a macro
  if (!testForDisableOrStop() && !runningMacro) {
    //* Circle: Startup and Home (Reset or Zero Turret)
    if (dbCircle->debounceAndPressed(ps5.Circle())) {
      if (!initialized) {
        reset();
      } else {
        zeroTurret();
      }
    }
    //* Triangle: Macro 1 - load from center
    else if (dbTriangle->debounceAndPressed(ps5.Triangle())) {
      loadFromCenter();
    }
    //* Cross: Macro 2 - handoff to runningback
    else if (dbCross->debounceAndPressed(ps5.Cross())) {
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
      if (QB_AUTO_ENABLED && dbOptions->debounceAndPressed(ps5.Options())) {
        switchMode();
      }
      //* Left Button (L1): Switch Target to Receiver 1
      else if (QB_AUTO_ENABLED && ps5.L1()) {
        switchTarget(receiver_1);
      }
      //* Right Button (R1): Switch Target to Receiver 2
      else if (QB_AUTO_ENABLED && ps5.R1()) {
        switchTarget(receiver_2);
      }
      //* Auto Mode
      else if (QB_AUTO_ENABLED && mode == automatic) {
        // TODO: Implement auto mode
        // do something based on current value of 'targetReceiver'
      }
      //* Manual Controls
      else {
        stickFlywheel = (ps5.LStickY() / 127.5f);
        stickTurret = (ps5.RStickX() / 127.5f);  

        //* Right Stick X: Turret Control
        // Left = CCW, Right = CW
        if (fabs(stickTurret) > STICK_DEADZONE) {
          setTurretSpeed(stickTurret * QB_TURRET_STICK_SCALE_FACTOR);
        } else {
          setTurretSpeed(0);
        }

        // updateTurretMotionStatus();

        //* Left Stick Y: Flywheel Override
        if (fabs(stickFlywheel) > STICK_DEADZONE) {
          setFlywheelSpeed(stickFlywheel);
        } else {
          //* D-Pad Up: Increase flywheel speed by one stage
          if (dbDpadUp->debounceAndPressed(ps5.Up())) {
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

// note that because the direction is flipped to be more intuitive for the driver,
// the "positive" direction is reversal/red on the falcon, and the "negative" direction is forwards/green
// positive direction is also positive encoder direction, and vice versa
void QuarterbackTurret::setTurretSpeed(float absoluteSpeed, bool overrideEncoderTare = false) {
  if (enabled) {
    targetTurretSpeed = constrain(absoluteSpeed, -1.0, 1.0);
    turretMotor.write(-targetTurretSpeed); // flip direction so that + is CW and - is CCW

    // handle mechanical slop when changing directions
    if (!overrideEncoderTare) {
      turretDirectionChanged();
    }

    currentTurretSpeed = targetTurretSpeed; //! for now, will probably need to change later, like an interrupt
  } else {
    turretMotor.write(0);
  }
}

void QuarterbackTurret::moveTurret(int heading, bool relativeToRobot) {
  moveTurret(heading, degrees, relativeToRobot);
}

void QuarterbackTurret::moveTurret(int heading, TurretUnits units, bool relativeToRobot) {
  Serial.print(F("moveTurret called with heading ="));
  Serial.print(heading);
  Serial.print(F(", units = "));
  if (units == degrees) { Serial.print(F("degrees, rel = ")); } else {Serial.print(F("counts, rel = ")); }
  Serial.println(relativeToRobot);
  if (enabled) {

    // todo
    if (relativeToRobot) {
      // todo: use encoder + laser to determine position
      targetRelativeHeading = heading;
      if (units == degrees) {
        moveTurret(heading, counts, relativeToRobot);
      } else if (units == counts) {
        targetTurretEncoderCount = heading * QB_COUNTS_PER_TURRET_DEGREE;
        turretMoving = true;
        setTurretSpeed(0.2 * copysign(1, stickTurret)); //! temp magic number, will eventually need to ramp
        // currentTurretEncoderCount = targetTurretEncoderCount; // currentTurretEncoderCount is updated by interrupt
      }
      currentRelativeHeading = targetRelativeHeading;
    } else { // relative to field
      // todo: use magnetometer
    }
  } else {
    setTurretSpeed(0);
  }
}

void QuarterbackTurret::moveTurretAndWait(int heading, bool relativeToRobot) {
  moveTurret(heading, relativeToRobot);
  while (turretMoving) {
    updateTurretMotionStatus();
    delay(10);
  }
}

void QuarterbackTurret::updateTurretMotionStatus() {
  if (turretMoving && fabs(currentTurretEncoderCount - targetTurretEncoderCount) < QB_TURRET_THRESHOLD) {
    turretMoving = false;
    if (fabs(stickTurret) < STICK_DEADZONE) {
      setTurretSpeed(0);
    }
  }
}

void QuarterbackTurret::turretDirectionChanged() {
  if (currentTurretSpeed > 0 && targetTurretSpeed < 0) { // going CW, trying to go CCW
    currentTurretEncoderCount += slopError;
    targetTurretEncoderCount += slopError;
  } else if (currentTurretSpeed < 0 && targetTurretSpeed > 0) { // going CCW, trying to go CW
    currentTurretEncoderCount -= slopError;
    targetTurretEncoderCount -= slopError;
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

void QuarterbackTurret::moveCradleSubroutine() {
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

void QuarterbackTurret::moveCradle(CradleState state, bool force) {
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

      if (targetCradleState != currentCradleState || force) {
        moveCradleSubroutine();
      }

      //* force is a blocking routine to ensure it works without interruption
      //* do not use force frequently as it can strain the actuator
      //* this should only be used on startup
      if (force) {
        // also allow emergency stop
        while ((millis() - cradleStartTime) <= QB_CRADLE_TRAVEL_DELAY && !testForDisableOrStop()) {
          NOP();
        }
        currentCradleState = targetCradleState;
        cradleMoving = false;
        cradleActuator.write(0);
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

void QuarterbackTurret::setFlywheelSpeed(float absoluteSpeed) {
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
  moveTurretAndWait(180);
  moveCradle(forward);
  this->runningMacro = false;
}

void QuarterbackTurret::zeroTurret() {
  this->runningMacro = true;
  Serial.println(F("zero called"));
  Serial.print(F("STARTING count: "));
  Serial.println(currentTurretEncoderCount);
  Serial.println(F("Resetting count to 0"));
  currentTurretEncoderCount = 0;
  targetRelativeHeading = 0;

  // set speed
  setTurretSpeed(QB_HOME_PCT);

  // pin will only read high if the main power is off or the laser sensor is triggered
  while (
    digitalRead(turretLaserPin) == LOW && // routine will exit when the laser sensor is triggered
    currentTurretEncoderCount < (2 * QB_COUNTS_PER_TURRET_REV) && // routine will exit if it spins around twice without triggering
    !testForDisableOrStop() // routine will exit if emergency stop or disable buttons are triggered
  ) {
    Serial.print(F("zeroing, read = "));
    Serial.print(digitalRead(turretLaserPin));
    Serial.print(F("; cte_count: "));
    Serial.println(currentTurretEncoderCount);
  }

  Serial.print(F("laser should read high rn: read = "));
  Serial.println(digitalRead(turretLaserPin));

  // get values as soon as the laser reads high
  int32_t risingEdgeEncoderCount = currentTurretEncoderCount;
  int32_t risingEdgeTimestamp = millis();

  while (
    digitalRead(turretLaserPin) == HIGH && // exit when the laser sensor goes low
    (currentTurretEncoderCount - risingEdgeEncoderCount) < (QB_COUNTS_PER_TURRET_REV / 2) && // exit if traveled half a rotation without triggering
    !testForDisableOrStop() // exit if emergency stop or disable buttons are triggered
  ) {
    Serial.print(F("zeroing (stage 2), read = "));
    Serial.print(digitalRead(turretLaserPin));
    Serial.print(F("; cte_count: "));
    Serial.println(currentTurretEncoderCount);
  }

  Serial.print(F("laser should read low rn: read = "));
  Serial.println(digitalRead(turretLaserPin));

  int32_t fallingEdgeEncoderCount = currentTurretEncoderCount;
  int32_t fallingEdgeTimestamp = millis();

  // stop turret
  setTurretSpeed(0);

  Serial.print(F("rising: count: "));
  Serial.print(risingEdgeEncoderCount);
  Serial.print(F(", time: "));
  Serial.println(risingEdgeTimestamp);
  Serial.print(F("falling: count: "));
  Serial.print(fallingEdgeEncoderCount);
  Serial.print(F(", time: "));
  Serial.println(fallingEdgeTimestamp);
  
  // Here lastTurretEncoderCount is used as the value of currentTurretEncoderCount in the previous iteration of the loop
  int32_t lastTurretEncoderCount = -currentTurretEncoderCount; // just something different than current
  uint16_t stopCounter = 0;

  // wait until turret is stopped
  while (
    stopCounter < (QB_TURRET_STOP_THRESHOLD_MS / QB_TURRET_STOP_LOOP_DELAY_MS) &&
    !testForDisableOrStop()
  ) {
    // run a counter for how many loop iterations that the last and current are the same.
    // if they are the same for a predetermined amount of time (QB_TURRET_STOP_THRESHOLD_MS),
    // we assume that the motor has actually stopped.
    if (lastTurretEncoderCount == currentTurretEncoderCount) {
      stopCounter++;
    } else {
      stopCounter = 0;
    }
    
    Serial.print(F("last ct: "));
    Serial.print(lastTurretEncoderCount);
    Serial.print(F(", current ct: "));
    Serial.print(currentTurretEncoderCount);
    Serial.print(F(", stopCt: "));
    Serial.println(stopCounter);

    lastTurretEncoderCount = currentTurretEncoderCount; // update last count

    delay(QB_TURRET_STOP_LOOP_DELAY_MS); // then delay to wait for encoder to update
  }

  int32_t restEncoderCount = currentTurretEncoderCount;
  int32_t restTimestamp = millis();

  // at this point, there should be 3 points of data, in increasing order as follows:
  //  - the point at which the laser was first triggered (when it first went high)
  //  - the point at which the laser stopped being triggered (when it went low after being high)
  //  - the point at which the motor stopped moving after being commanded to stop
  
  // the first point is the point of reference.
  // the middle of the first and second points is the target point, where we assume the true zero is.
  // the third point tells us how far we are from the second point, i.e., the error caused by the motor not stopping perfectly.
  //  - this is not needed between the first and second points because the motor does not stop moving.
  
  // the third point also helps us overcome the mechanical slop issue when changing directions on the turret, 
  // since we assume that the turret rotates outside the laser triggering range when it is told to stop.
  // so, we start rotating in the other direction, then when the laser triggers again, we can account for the slop
  // near-perfectly by forcing the current encoder count to the second point when the laser first triggered, 
  // then continue moving until reaching the halfway point PLUS the difference between the second and third point,
  // the latter of which is the number of counts the motor took to stop, so that it should stop exactly on the halfway mark.

  // now, to actually do this, we start moving the motor (which was stopped), but in the opposite direction.

  Serial.println(F("motor stopped, now moving in opposite direction"));

  setTurretSpeed(-QB_HOME_PCT);

  // moving with a positive power increases the current encoder count, and vice versa
  // since we are moving with a negative power, the encoder count will be decreasing

  // wait for the laser to trigger (go high) again, then measure the difference 
  // between the current encoder count and the known falling edge count.
  // this value represents the mechanical slop, which can be used later.
  // after that, tare the value of the current encoder count to the falling edge count.
  
  while (
    digitalRead(turretLaserPin) == LOW && // exit when the laser sensor is triggered
    !testForDisableOrStop() // exit if emergency stop or disable buttons are triggered
  ) {
    Serial.print(F("zeroing (stage 3), read = "));
    Serial.print(digitalRead(turretLaserPin));
    Serial.print(F("; cte_count: "));
    Serial.print(currentTurretEncoderCount);
    Serial.print(F("; fall_ct: "));
    Serial.println(fallingEdgeEncoderCount);
    delay(5);
  }

  // at this point, we will record the difference between the current count (physically, at the second point or falling edge) 
  // and the rest count (third point or stopping point). the current encoder count should be less than the falling edge count 
  // (past it, if it were to be physically translated) due to the mechanical slop
  // int32_t reEntryEncoderCount = currentTurretEncoderCount;
  // int32_t reEntryTimestamp = millis();

  // the error only due to the motor not stopping perfectly is then found by the difference between the first and third points
  stopError = restEncoderCount - risingEdgeEncoderCount;

  // calculate the error due to slop
  slopError = fallingEdgeEncoderCount - currentTurretEncoderCount;


  Serial.print(F("stop error: "));
  Serial.print(stopError);
  Serial.print(F("; slop error: "));
  Serial.println(slopError);

  // then, we tare the current count to the third point (falling edge), since we assume it is there
  currentTurretEncoderCount = fallingEdgeEncoderCount;

  // find the theoretical midpoint, then add the stop error to get the target count
  // int32_t targetCount = ((fallingEdgeEncoderCount + risingEdgeEncoderCount) / 2) - stopError; // for if we change directions again
  int32_t targetCount = ((fallingEdgeEncoderCount + risingEdgeEncoderCount) / 2) + (stopError * QB_TURRET_HOME_STOP_FACTOR);

  Serial.print(F("target count: "));
  Serial.println(targetCount);

  // finally, move to the target count, then stop
  // setTurretSpeed(QB_HOME_PCT);


  while (
    // currentTurretEncoderCount < targetCount && 
    currentTurretEncoderCount > targetCount && 
    !testForDisableOrStop()
  ) {
    Serial.print(F("zeroing (stage 4), read = "));
    Serial.print(digitalRead(turretLaserPin));
    Serial.print(F("; cte_count: "));
    Serial.print(currentTurretEncoderCount);
    Serial.print(F("; target_ct: "));
    Serial.print(targetCount);
    Serial.print(F("; current_ct > target_ct? = "));
    Serial.println(currentTurretEncoderCount > targetCount);
    delay(5);
  }

  // stop turret and tare everything
  setTurretSpeed(0);
  currentRelativeHeading = 0;
  currentTurretEncoderCount = 0;
  Serial.println(F("zeroed"));
  this->runningMacro = false;
}

void QuarterbackTurret::reset() {
  this->enabled = true;
  this->runningMacro = true;
  moveCradle(back, true); // force
  // loadFromCenter();
  zeroTurret(); // temp: just zero
  this->initialized = true;
  this->runningMacro = false;
}

bool QuarterbackTurret::testForDisableOrStop() {
  //* Touchpad: Emergency Stop
  if (ps5.Touchpad()) {
    emergencyStop();
    Serial.println(F("emergency stopping"));
    return true;
  }
  //* Square: Toggle Flywheels/Turret On/Off (Safety Switch)
  else if (dbSquare->debounceAndPressed(ps5.Square())) {
    if (!enabled) {
      setEnabled(true);
      Serial.println(F("setting enabled"));
    } else {
      setEnabled(false);
      Serial.println(F("setting disabled"));
    }
    return true;
  } 
  else {
    // Serial.println(F("not disabling or stopping"));
    return false;
  }
}

void QuarterbackTurret::setEnabled(bool enabled) {
  this->enabled = enabled;
}

void QuarterbackTurret::emergencyStop() {
  this->enabled = false;
  setFlywheelSpeed(0); // this will not change the state variables since the bot is disabled
  setTurretSpeed(0);
  cradleActuator.write(0);
  // TODO: stop assembly stepper motor
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
  if (enabled) {
    Serial.print(F("turretLaserState: "));
    Serial.print(digitalRead(turretLaserPin));
    Serial.print(F("; currentTurretEncoderCount: "));
    Serial.println(currentTurretEncoderCount);
  }
}