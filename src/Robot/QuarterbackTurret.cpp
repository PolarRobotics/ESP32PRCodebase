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
  uint8_t flywheelLeftPin,    // M1
  uint8_t flywheelRightPin,   // M2
  uint8_t cradlePin,          // M3
  uint8_t turretPin,          // M4
  uint8_t assemblyStepPin,    // S1
  uint8_t assemblyDirPin,     // S2 
  uint8_t magnetometerSdaPin, // S3
  uint8_t magnetometerSclPin, // S4
  uint8_t turretEncoderPinA,  // E1A
  uint8_t turretEncoderPinB,  // E1B
  uint8_t turretLaserPin      // E2A
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
  
  this->magnetometerHeading = 0;

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

  magnetometerSetup();
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
    else if (ps5.Left()) {
      testRoutine();
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
          calculateHeadingMag();
        } else {
          //setTurretSpeed(0);
          calculateHeadingMag();
          holdTurretStill();
          updateTurretMotionStatus();
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
void QuarterbackTurret::setTurretSpeed(float absoluteSpeed, bool overrideEncoderTare) {
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

void QuarterbackTurret::moveTurret(int16_t heading, bool relativeToRobot) {
  moveTurret(heading, degrees, relativeToRobot);
}

void QuarterbackTurret::moveTurret(int16_t heading, TurretUnits units, bool relativeToRobot) {
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
        setTurretSpeed(QB_HOME_PCT * copysign(1, heading)); //! temp constant, implement P loop soon
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

void QuarterbackTurret::moveTurretAndWait(int16_t heading, bool relativeToRobot) {
  moveTurret(heading, relativeToRobot);
  while (turretMoving && !testForDisableOrStop()) {
    updateTurretMotionStatus();
    delay(10);
  }
}

void QuarterbackTurret::updateTurretMotionStatus() {
  // Serial.print(F("update called with ctec = "));
  // Serial.print(currentTurretEncoderCount);
  // Serial.print(F("; ttec = "));
  // Serial.println(targetTurretEncoderCount);
  // determines if encoder is within "spec"
  if (turretMoving && fabs((currentTurretEncoderCount % QB_COUNTS_PER_TURRET_REV) - targetTurretEncoderCount) < QB_TURRET_THRESHOLD) {
    turretMoving = false;
    setTurretSpeed(0);
  }
}

void QuarterbackTurret::turretDirectionChanged() {
  if (currentTurretSpeed > 0 && targetTurretSpeed < 0) { // going CW, trying to go CCW
    currentTurretEncoderCount -= slopError;
    targetTurretEncoderCount -= slopError;
  } else if (currentTurretSpeed < 0 && targetTurretSpeed > 0) { // going CCW, trying to go CW
    currentTurretEncoderCount += slopError;
    targetTurretEncoderCount += slopError;
  }
}

int16_t QuarterbackTurret::getCurrentHeading() {
  return (currentTurretEncoderCount / QB_COUNTS_PER_TURRET_DEGREE) % 360;
}

int16_t QuarterbackTurret::findNearestHeading(int16_t targetHeading, int16_t currentHeading) {
  // assuming targetHeading is positive
  int16_t positiveHeading = targetHeading;

  // if targetHeading is negative, convert to a positive heading
  if (targetHeading < 0) {
    positiveHeading = targetHeading + 360;
  }

  // properly constrain headings to be within (-360, +360)
  positiveHeading %= 360;
  int16_t negativeHeading = positiveHeading - 360;
  if (negativeHeading == -360) // same as if (positiveHeading == 0)
    negativeHeading = 0; 

  // calculate which heading is closer to the current heading
  int16_t adjustedCurrentHeading = currentHeading % 360;

  if (abs(adjustedCurrentHeading - positiveHeading) < abs(adjustedCurrentHeading - negativeHeading)) {
    // negative heading is closer
    return negativeHeading;
  } else {
    // positive heading is closer
    return positiveHeading;
  }
}

int16_t QuarterbackTurret::findNearestHeading(int16_t targetHeading) {
  findNearestHeading(targetHeading, currentRelativeHeading);
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
  // zeroTurret();
  this->runningMacro = false;
}

void QuarterbackTurret::handoff() {
  this->runningMacro = true;
  aimAssembly(straight);
  int16_t targetHeading = (getCurrentHeading() + 180) % 360;
  moveTurretAndWait(targetHeading);
  // moveTurretAndWait(180);
  setFlywheelSpeedStage(slow_outwards);
  moveCradle(forward);
  this->runningMacro = false;
}

void QuarterbackTurret::testRoutine() {
  this->runningMacro = true;
  Serial.println(F("test routine called"));
  Serial.println(F("initial ctec: "));
  Serial.println(currentTurretEncoderCount);
  moveTurretAndWait(90);
  delay(500);
  Serial.println(F("ctec after turn to 90 deg: "));
  Serial.println(currentTurretEncoderCount);
  moveTurretAndWait(-90);
  delay(500);
  Serial.println(F("ctec after turn to -90 deg: "));
  Serial.println(currentTurretEncoderCount);
  moveTurretAndWait(180);
  delay(500);
  Serial.println(F("ctec after turn to 180 deg: "));
  Serial.println(currentTurretEncoderCount);
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

  //Now that the encoder is zeroed we can just zero the magnetometer
  delay(250);
  calibMagnetometer();

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
    /*
    Serial.print(F("turretLaserState: "));
    Serial.print(digitalRead(turretLaserPin));
    Serial.print(F("; currentTurretEncoderCount: "));
    Serial.println(currentTurretEncoderCount);
    */
  }
}

/**
 * @brief Sets up magnetometer
 * @authors Rhys Davies, Corbin Hibler
 * @date 2024-01-03
*/
void QuarterbackTurret::magnetometerSetup() {
  //* MAGNETOMETER STUFF
  if (! lis3mdl.begin_I2C()) {          // hardware I2C mode, can pass in address & alt Wire
  //if (! lis3mdl.begin_SPI(LIS3MDL_CS)) {  // hardware SPI mode
  //if (! lis3mdl.begin_SPI(LIS3MDL_CS, LIS3MDL_CLK, LIS3MDL_MISO, LIS3MDL_MOSI)) { // soft SPI
    Serial.println("Failed to find LIS3MDL chip");
  }
  Serial.println("LIS3MDL Found!");

  lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  Serial.print("Performance mode set to: ");
  switch (lis3mdl.getPerformanceMode()) {
    case LIS3MDL_LOWPOWERMODE: Serial.println("Low"); break;
    case LIS3MDL_MEDIUMMODE: Serial.println("Medium"); break;
    case LIS3MDL_HIGHMODE: Serial.println("High"); break;
    case LIS3MDL_ULTRAHIGHMODE: Serial.println("Ultra-High"); break;
  }

  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  Serial.print("Operation mode set to: ");
  // Single shot mode will complete conversion and go into power down
  switch (lis3mdl.getOperationMode()) {
    case LIS3MDL_CONTINUOUSMODE: Serial.println("Continuous"); break;
    case LIS3MDL_SINGLEMODE: Serial.println("Single mode"); break;
    case LIS3MDL_POWERDOWNMODE: Serial.println("Power-down"); break;
  }

  lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
  // You can check the datarate by looking at the frequency of the DRDY pin
  Serial.print("Data rate set to: ");
  switch (lis3mdl.getDataRate()) {
    case LIS3MDL_DATARATE_0_625_HZ: Serial.println("0.625 Hz"); break;
    case LIS3MDL_DATARATE_1_25_HZ: Serial.println("1.25 Hz"); break;
    case LIS3MDL_DATARATE_2_5_HZ: Serial.println("2.5 Hz"); break;
    case LIS3MDL_DATARATE_5_HZ: Serial.println("5 Hz"); break;
    case LIS3MDL_DATARATE_10_HZ: Serial.println("10 Hz"); break;
    case LIS3MDL_DATARATE_20_HZ: Serial.println("20 Hz"); break;
    case LIS3MDL_DATARATE_40_HZ: Serial.println("40 Hz"); break;
    case LIS3MDL_DATARATE_80_HZ: Serial.println("80 Hz"); break;
    case LIS3MDL_DATARATE_155_HZ: Serial.println("155 Hz"); break;
    case LIS3MDL_DATARATE_300_HZ: Serial.println("300 Hz"); break;
    case LIS3MDL_DATARATE_560_HZ: Serial.println("560 Hz"); break;
    case LIS3MDL_DATARATE_1000_HZ: Serial.println("1000 Hz"); break;
  }
  
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
  Serial.print("Range set to: ");
  switch (lis3mdl.getRange()) {
    case LIS3MDL_RANGE_4_GAUSS: Serial.println("+-4 gauss"); break;
    case LIS3MDL_RANGE_8_GAUSS: Serial.println("+-8 gauss"); break;
    case LIS3MDL_RANGE_12_GAUSS: Serial.println("+-12 gauss"); break;
    case LIS3MDL_RANGE_16_GAUSS: Serial.println("+-16 gauss"); break;
  }

  lis3mdl.setIntThreshold(500);
  lis3mdl.configInterrupt(false, false, true, // enable z axis
                          true, // polarity
                          false, // don't latch
                          true); // enabled!
}

float QuarterbackTurret::turretPIDController(int setPoint, float kp, float kd, float ki) {
  //Measure the time elapsed since last iteration
  long currentTime = millis();
  float deltaT = ((float)(currentTime - previousTime));

  //Only update the current PID values every 100 ms
  if (deltaT > 5 && deltaT < 25) {
    /*Computer the error, derivative, and integral*/
    //Note about Error: Distance from current position to requested position with respect to rollovers at 360 (makes it a bit more complicated)
    //Positive values of E will drive current heading towads 0
    // int normalDistance = headingdeg - targetAbsoluteHeading - 360;
    // int reverseDistance = headingdeg - targetAbsoluteHeading + 360;
    // normalDistance = normalDistance%360;
    // reverseDistance = reverseDistance%360;
    // int e = normalDistance;
    // if (abs(reverseDistance) < abs(normalDistance)) {
    //   e = reverseDistance;
    // }

    int e = findNearestHeading(headingdeg, targetAbsoluteHeading);

    //Find nearest heading

    float eDerivative = (e - ePrevious);
    eIntegral = eIntegral + e*.001;

    //Computer the PID control signal
    float u = (kp*e) + (ki * eIntegral) + (kd*eDerivative);

    //Divide by 100 and then constrain to be from 0 ot 1
    if (u>.2) {u=.2;}
    else if (u<-.2) { u = -.2;}

    //If the robot gets within 5 degrees then send error etc to 0
    if (abs(e) < 10) {
      e = 0;
      eDerivative = 0;
      eIntegral = 0;
      ePrevious = 0;
    }

    Serial.print("DeltaT:\t"); Serial.print(deltaT);
    Serial.print("\tP:\t"); Serial.print((kp*e), 4);
    Serial.print("\tI:\t"); Serial.print((ki * eIntegral), 4);
    //Serial.print("\tD:\t"); Serial.print((kd*eDerivative) , 4);
    Serial.print("\tPWM Value:\t"); Serial.print(u , 4);
    Serial.print("\tCurrent Heading [deg]:\t"); Serial.print(headingdeg , 0);
    Serial.print("\tTarget Heading [deg]:\t"); Serial.print(targetAbsoluteHeading , 0);
    Serial.println();

    //Update variables for next iteration
    previousTime = currentTime;
    ePrevious = e;
    return -u;
  } else if (deltaT > 250) {
    previousTime = currentTime;
    return turretPIDSpeed;
  } else {
    return turretPIDSpeed;
  }
  
}

void QuarterbackTurret::holdTurretStill() {

  if (magnetometerCalibrated) {
    turretPIDSpeed = turretPIDController(targetAbsoluteHeading, kp, kd, ki);
    setTurretSpeed(turretPIDSpeed);
    //Serial.print("PWM Signal:\t"); Serial.print(turretPIDSpeed);
    //Serial.println();
  }

  //Serial.print("Current Heading [deg]:\t"); Serial.print(headingdeg);
  //Serial.print("\tTarget Heading [deg]:\t"); Serial.print(targetAbsoluteHeading);
  //Serial.print("\tError [deg]:\t"); Serial.print(abs(headingError));
  //Serial.print("\tPWM Value:\t"); Serial.print(turretPIDSpeed);
  //Serial.println();
}

void QuarterbackTurret::calculateHeadingMag() {
  //Only run the code in here if the calibration has been done to the magnetometer
  if (magnetometerCalibrated) {
    lis3mdl.read(); 
    //Calculate the current angle of the turret based on the calibration data
    if (xsign) {
      xVal = lis3mdl.x + xHalf;
    } else {
      xVal = lis3mdl.x - xHalf;
    }

    if (ysign) {
      yVal = lis3mdl.y + yHalf;
    } else {
      yVal = lis3mdl.y - yHalf;
    }
    
    

    //Evaluate both ranges of X and Y then scale the smaller value to be within the same range as the larger
    if (yHalf > xHalf) {
      xVal = (double)((double)xVal/((double)xHalf))*(double)yHalf;
    } else if (xHalf > yHalf) {
      yVal = (double)((double)yVal/((double)yHalf))*(double)xHalf;
    }
    
    //Calculate angle in radians
    if (xVal !=-1 && xVal !=0 && yVal != 0 && yVal !=-1) {
      headingrad = atan2(yVal, xVal);
    }

    //Convert to degrees
    headingdeg = headingrad*180/M_PI;

    //If the degrees are negative then they just need inversed plus 180
    if (headingdeg < 0) {
      headingdeg+=360;
    }
    headingdeg = (int)headingdeg;

    prevTurretAngles[prevTurretAngleIndex] = headingdeg;
    prevTurretAngleIndex++;
    prevTurretAngleIndex %= std::end(prevTurretAngles) - std::begin(prevTurretAngles);

    //Taking the avergage for heading
    headingdeg = 0;
    for (int i = 0; i<(std::end(prevTurretAngles) - std::begin(prevTurretAngles)); i++) {
      headingdeg+=prevTurretAngles[i];
    }
    headingdeg/=(std::end(prevTurretAngles) - std::begin(prevTurretAngles));

    /*DEBUGGING PRINTOUTS*/
    //Serial.print("X:  "); Serial.print(lis3mdl.x); 
    //Serial.print("\tY:  "); Serial.print(lis3mdl.y); 
    //Serial.print("\tMinX:  "); Serial.print(minX); 
    //Serial.print("\tMaxX:  "); Serial.print(maxX); 
    //Serial.print("\tMinY:  "); Serial.print(minY); 
    //Serial.print("\tMaxY:  "); Serial.print(maxY); 
    //Serial.print("\txAdapt:  "); Serial.print(xVal);
    //Serial.print("\tyAdapt:  "); Serial.print(yVal);
    //Serial.print("\tHeading [deg]:   "); Serial.print(headingdeg);

    //Delay in program to make printouts readable
    //delay(50);
    //Serial.println();
    }
}

/*
  This will spin then turret around 360 degrees semi-slowly to allow for the magnetometer to calibrate itself wherever it is started up
*/
void QuarterbackTurret::calibMagnetometer() {
  
    int degreesMove = 360;
    targetTurretEncoderCount = degreesMove * QB_COUNTS_PER_TURRET_DEGREE;
    turretMoving = true;
    setTurretSpeed(QB_HOME_MAG * copysign(1, degreesMove));
    while (currentTurretEncoderCount < targetTurretEncoderCount && !testForDisableOrStop()){
      // get X Y and Z data at once
      lis3mdl.read();      

      //Constantly looking for min and max values of X
      if (lis3mdl.x < minX && lis3mdl.x != -1 && lis3mdl.x != 0) {minX = lis3mdl.x;}
      else if (lis3mdl.x > maxX && lis3mdl.x != -1 && lis3mdl.x != 0) {maxX = lis3mdl.x;}

      //Adjusting X values to range from + or - values rather than all positive
      xHalf = abs(maxX) - abs(minX);
      xHalf/= 2;
      xHalf+= abs(minX);

      //Constantly looking for min and max values of Y
      if (lis3mdl.y < minY && lis3mdl.y != -1 && lis3mdl.y != 0 && lis3mdl.y != 10) {minY = lis3mdl.y;}
      else if (lis3mdl.y > maxY && lis3mdl.y != -1 && lis3mdl.y != 0 && lis3mdl.y != 10) {maxY = lis3mdl.y;}

      //Adjusting Y values to range from + or - values rather than all positive
      yHalf = abs(maxY) - abs(minY);
      yHalf/= 2;
      yHalf+= abs(minY);
    
      /*DEBUGGING PRINTOUTS*/
      Serial.print("X:  "); Serial.print(lis3mdl.x); 
      Serial.print("\tY:  "); Serial.print(lis3mdl.y); 
      Serial.print("\tMinX:  "); Serial.print(minX); 
      Serial.print("\tMaxX:  "); Serial.print(maxX); 
      Serial.print("\tMinY:  "); Serial.print(minY); 
      Serial.print("\tMaxY:  "); Serial.print(maxY); 

    //Delay in program to make printouts readable
    //delay(50);  
    Serial.println();
    }

    //Updating variables that will be used to handle other two possible sign cases for each value
    if ((maxX+minX) < 0) {
      xsign = true;
    }
    if ((maxY+minY) < 0) {
      ysign = true;
    }

    magnetometerCalibrated = true;
    Serial.println("Magnetometer has been calibrated!");
    eIntegral = 0;
    previousTime = millis();
}

/**
 * @brief Returns value of class field magnetometerHeading.
 * @author Corbin Hibler
 * @date 2024-01-03
*/
int16_t QuarterbackTurret::getMagnetometerHeading() {
  return this->magnetometerHeading;
}