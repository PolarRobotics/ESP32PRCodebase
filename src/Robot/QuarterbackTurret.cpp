#include "QuarterbackTurret.h"

#define RECEIVER1_ID 0xDECAB667E510DD09ULL
#define RECEIVER2_ID 0xDECA114583701E85ULL

//This for some reason has to be declared in the .cpp file and not the .h file so that it does not conflict with the same declaration in other .h files
// HardwareSerial Uart_Turret(2); // UART2

// "define" static members to satisfy linker
uint8_t QuarterbackTurret::turretEncoderPinA;
uint8_t QuarterbackTurret::turretEncoderPinB;
uint8_t QuarterbackTurret::turretEncoderStateB;
uint8_t QuarterbackTurret::turretLaserPin;
int32_t QuarterbackTurret::currentTurretEncoderCount;

double rx;
double ry;

void QuarterbackTurret::turretEncoderISR() {
  turretEncoderStateB = digitalRead(turretEncoderPinB);
  if (turretEncoderStateB == 1) {
    currentTurretEncoderCount--;
  } else if (turretEncoderStateB == 0) {
    currentTurretEncoderCount++;
  }
  // if(digitalRead(turretLaserPin) == HIGH){
  //   currentTurretEncoderCount = 0; // reset the encoder count when the laser is triggered
  // }
}

#pragma region Constructor
QuarterbackTurret::QuarterbackTurret(
  uint8_t flywheelLeftPin, // M1
  uint8_t flywheelRightPin, // M2
  uint8_t cradlePin, // M3
  uint8_t turretPin, // M4
  uint8_t assemblyPin, // S1
  uint8_t magnetometerSdaPin, // S3
  uint8_t magnetometerSclPin, // S4
  uint8_t turretEncoderPinA, // E1A
  uint8_t turretEncoderPinB, // E1B
  uint8_t turretLaserPin // E2A
) {
  // set all state variables to default values,
  // except currentAssemblyAngle, currentRelativeHeading, currentRelativeTurretCount
  // the positions of these mechanisms are initially unknown and assigned upon reset/homing
  this->enabled = false; // initially disable robot for safety
  this->initialized = false;
  this->runningMacro = false;
  this->currentAssemblyAngle = unknownAngle;
  this->targetAssemblyAngle = straight; // while the initial state is unknown, we want it to be straight
  this->assemblyMoving = false; // it is safe to assume the assembly is not moving
  this->assemblyTriggerToggled = false;
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
  this->manualHeadingIncrementCount = 0;
  this->currentAbsoluteHeading = 0;
  this->targetAbsoluteHeading = 0;
  this->stickFlywheel = 0;
  this->stickTurret = 0;

  // turret laser setup
  // QuarterbackTurret::turretLaserPin = turretLaserPin;
  // this->turretLaserState = 0;
  // pinMode(turretLaserPin, INPUT_PULLUP); //! will be 1 when at home position or main power is off (the latter is electrically unavoidable)

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
  assemblyMotor.setup(assemblyPin, small_12v);
  flywheelLeftMotor.setup(flywheelLeftPin, falcon);
  flywheelRightMotor.setup(flywheelRightPin, falcon);

  // initialize debouncers
  this->dbShare = new Debouncer(QB_BASE_DEBOUNCE_DELAY);
  this->dbOptions = new Debouncer(QB_BASE_DEBOUNCE_DELAY);
  this->dbSquare = new Debouncer(QB_BASE_DEBOUNCE_DELAY);
  this->dbDpadUp = new Debouncer(QB_BASE_DEBOUNCE_DELAY);
  this->dbDpadDown = new Debouncer(QB_BASE_DEBOUNCE_DELAY);
  this->dbDpadLeft = new Debouncer(QB_BASE_DEBOUNCE_DELAY);
  this->dbDpadRight = new Debouncer(QB_BASE_DEBOUNCE_DELAY);
  this->dbCircle = new Debouncer(QB_CIRCLE_HOLD_DELAY);
  this->dbTriangle = new Debouncer(QB_TRIANGLE_HOLD_DELAY);
  this->dbCross = new Debouncer(QB_BASE_DEBOUNCE_DELAY);
  this->dbTurretInterpolator = new Debouncer(QB_TURRET_INTERPOLATION_DELAY);

  magnetometerSetup();
  this->northHeadingDegrees = 7;

  // Initialize receiver positions to safe defaults to avoid garbage math
  for (int i = 0; i < NUM_RECEIVERS; i++) {
    receivers[i].position[0] = 1.0; // 1 meter default X
    receivers[i].position[1] = 3.0; // 3 meters default Y (straight ahead)
    receivers[i].distance = 0;
    receivers[i].angle = 0;
  }

  currReceiver = 0;

  // Uart_Turret.begin(115200, SERIAL_8N1, RX2, TX2);
  Serial2.begin(115200, SERIAL_8N1, RX2, TX2);
}
#pragma endregion

#pragma region action()
void QuarterbackTurret::action() {
  static unsigned long lastPrintTime = 0;
  static unsigned long lastStickActivity = 0;   // ← NEW: tracks right-stick activity for 1-second timeout

  //! Control Schema
  //* Touchpad: Emergency Stop
  //* Square: Toggle Flywheels/Turret On/Off (Safety Switch)
  if (!testForDisableOrStop() && !runningMacro) {
    //* Circle: Startup and Home
    if (dbCircle->debounceAndPressed(ps5.Circle())) {
      if (!initialized) {
        reset();
      } else {
        zeroTurret();
      }
    }
    //* Triangle: load from center
    else if (dbTriangle->debounceAndPressed(ps5.Triangle())) {
      loadFromCenter();
    }
    //* Cross: handoff to runningback
    else if (dbCross->debounceAndPressed(ps5.Cross())) {
      handoff();
    }
    else {
      // ─────────────────────────────────────────────────────────────
      // Always read both sticks (needed for auto-mode override)
      // ─────────────────────────────────────────────────────────────
      stickFlywheel = (ps5.LStickY() / 127.5f);
      stickTurret   = (ps5.RStickX() / 127.5f);

      //* Right Trigger (R2): Fire cradle
      if (currentFlywheelSpeed > STICK_DEADZONE && ps5.R2()) {
        moveCradle(forward);
      } else {
        moveCradle(back);
      }

      //* Left Trigger (L2): Toggle Assembly Angle
      if (ps5.L2() && !assemblyTriggerToggled) {
        assemblyTriggerToggled = true;
        if (currentAssemblyAngle == unknownAngle || currentAssemblyAngle == angled) {
          aimAssembly(straight);
        } else if (currentAssemblyAngle == straight) {
          aimAssembly(angled);
        }
      } else if (assemblyTriggerToggled && !ps5.L2()) {
        assemblyTriggerToggled = false;
      } else {
        aimAssembly(targetAssemblyAngle);
      }

      //* Share Button: Switch to Combine Mode
      if (dbShare->debounceAndPressed(ps5.Share())) {
        if (mode != combine) {
          switchMode(combine);
          firstCombine = true;
          this->combinePosition = combineStraight;
          targetRelativeHeading = 0;
          zeroTurret();
        } else {
          switchMode(manual);
        }
      }
      //* Options Button: Toggle Auto / Manual
      else if (QB_AUTO_ENABLED && dbOptions->debounceAndPressed(ps5.Options())) {
        switchMode();
      }
      //* L1 / R1: Switch receiver
      else if (QB_AUTO_ENABLED && ps5.L1()) {
        currReceiver--;
        if (currReceiver < 0) currReceiver = 0;
      }
      else if (QB_AUTO_ENABLED && ps5.R1()) {
        currReceiver++;
        if (currReceiver > NUM_RECEIVERS - 1) currReceiver = NUM_RECEIVERS - 1;
      }

      //* AUTO MODE with right-stick manual override
      else if (QB_AUTO_ENABLED && mode == automatic) {
        // Update receiver data (EMA)
        if (magnetometerCalibrated && newData) {
          double newRx = receivers[currReceiver].position[0];
          double newRy = receivers[currReceiver].position[1];

          if (posBufferCount == 0) {
            avgRx = newRx;
            avgRy = newRy;
          } else {
            avgRx = emaAlpha * newRx + (1 - emaAlpha) * avgRx;
            avgRy = emaAlpha * newRy + (1 - emaAlpha) * avgRy;
          }
          posBufferCount = 1;
          newData = false;
        }

        // ─────────────────────────────────────────────────────────────
        // RIGHT STICK MANUAL OVERRIDE (only in Auto mode)
        // ─────────────────────────────────────────────────────────────
        if (fabs(stickTurret) > STICK_DEADZONE) {
          // User is moving the stick → fine manual adjustment
          float overrideSpeed = stickTurret * 0.12;
          setTurretSpeed(overrideSpeed);
          lastStickActivity = millis();
        }
        else if (millis() - lastStickActivity < 2000) {
          // Stick just returned to center → stop turret for 1 second
          setTurretSpeed(0);
        }
        else {
          // No stick input for 1+ second → resume normal auto tracking
          if (fabs(avgRx) > 0.01 || fabs(avgRy) > 0.01) {
            calculateHeadingMag();
            currentRelativeHeading = headingDeg;
            double tempRx = rx, tempRy = ry;
            rx = avgRx; ry = avgRy;
            targetRelativeHeading = angleToTarget(receivers[currReceiver]);
            turretPIDSpeed = turretPIDController((float)currentRelativeHeading,
                                                 (float)targetRelativeHeading,
                                                 kp, kd, ki, 0.1);
            setTurretSpeed(turretPIDSpeed);

            // Flywheel update every 1 s
            unsigned long now = millis();
            if (now - lastFlywheelUpdate >= 1000) {
              setAutoFlywheelSpeed(0);
              lastFlywheelUpdate = now;
            }
            rx = tempRx; ry = tempRy;
          }
        }
      }

      //* MANUAL / COMBINE MODE (unchanged)
      else {
        if (mode == combine) {
          if (firstCombine) {
            firstCombine = false;
            combineMoveRight();
            aimAssembly(angled);
          }

          if (dbDpadLeft->debounceAndPressed(ps5.Left())) {
            combineMoveLeft();
          }
          else if (dbDpadRight->debounceAndPressed(ps5.Right())) {
            combineMoveRight();
          }

          if (abs(CalculateRotation(getCurrentHeading(), targetRelativeHeading)) >= 2) {
            turretPIDSpeed = turretPIDController((float)getCurrentHeading(),
                                                 (float)targetRelativeHeading,
                                                 kp, kd, ki, 0.2);
            setTurretSpeed(turretPIDSpeed);
          }
        } else {
          // Right stick turret control (manual mode)
          if (fabs(stickTurret) > STICK_DEADZONE) {
            if (useMagnetometer && holdTurretStillEnabled) {
              if (manualHeadingIncrementCount == 0) {
                targetAbsoluteHeading += (1 * copysign(1, stickTurret));
                targetAbsoluteHeading %= 360;
              } else {
                manualHeadingIncrementCount++;
                manualHeadingIncrementCount %= 4;
              }
              calculateHeadingMag();
              holdTurretStill();
            } else {
              setTurretSpeed(stickTurret * QB_TURRET_STICK_SCALE_FACTOR);
            }
          } else {
            if (useMagnetometer && holdTurretStillEnabled) {
              calculateHeadingMag();
              holdTurretStill();
            } else {
              setTurretSpeed(0);
            }
          }
        }

        updateTurretMotionStatus();

        //* Left Stick Y + D-Pad: Flywheel control
        if (mode != automatic) {
          if (fabs(stickFlywheel) > STICK_DEADZONE) {
            setFlywheelSpeed(stickFlywheel);
          } else {
            if (dbDpadUp->debounceAndPressed(ps5.Up())) {
              float newSpeed = currentFlywheelSpeed + 0.025;
              setFlywheelSpeed(newSpeed);
            }
            else if (dbDpadDown->debounceAndPressed(ps5.Down())) {
              float newSpeed = currentFlywheelSpeed - 0.025;
              setFlywheelSpeed(newSpeed);
            }
          }
        }
      }
    }
  }

  readTargetingInfo();
  delay(5);
  printDebug();

  // Periodic debug print (you can remove this if you want total silence)
  if (millis() - lastPrintTime >= 1000) {
    calculateHeadingMag();
    Serial.print("Magnetometer angle: ");
    Serial.println(headingDeg);

    Serial.print("QB Position: x=");
    Serial.print(position[0]);
    Serial.print(", y=");
    Serial.println(position[1]);

    lastPrintTime = millis();
  }
}
#pragma endregion

// note that because the direction is flipped to be more intuitive for the driver,
// the "positive" direction is reversal/red on the falcon, and the "negative" direction is forwards/green
// positive direction is also positive encoder direction, and vice versa
// note that because the direction is flipped to be more intuitive for the driver,
// the "positive" direction is reversal/red on the falcon, and the "negative" direction is forwards/green
// positive direction is also positive encoder direction, and vice versa
// note that because the direction is flipped to be more intuitive for the driver,
// the "positive" direction is reversal/red on the falcon, and the "negative" direction is forwards/green
// positive direction is also positive encoder direction, and vice versa
void QuarterbackTurret::setTurretSpeed(float absoluteSpeed, bool overrideEncoderTare) {
  if (enabled) {
    targetTurretSpeed = constrain(absoluteSpeed, -1.0, 1.0);

    // This is the actual value sent to the motor after the built-in flip
    float pwm = -targetTurretSpeed;

    // Boost ONLY counterclockwise (which becomes positive PWM after the flip)
    if (pwm > 0) {                          // ← counterclockwise command
      pwm += QB_CCW_SPEED_BOOST;            // increase power for CCW
      pwm = constrain(pwm, 0.0f, 1.0f);
    }

    turretMotor.write(pwm);

    currentTurretSpeed = targetTurretSpeed;   // keep original value for logic
  } else {
    turretMotor.write(0);
  }
}

#pragma region Old Rel Turret
void QuarterbackTurret::moveTurret(int16_t heading, bool relativeToRobot, bool ramp) {
  moveTurret(heading, degrees, QB_HOME_PCT, relativeToRobot, ramp);
}

void QuarterbackTurret::moveTurret(int16_t heading, float power, bool relativeToRobot, bool ramp) {
  moveTurret(heading, degrees, power, relativeToRobot, ramp);
}

void QuarterbackTurret::moveTurret(int16_t heading, TurretUnits units, float power, bool relativeToRobot, bool ramp) {
  Serial.print(F("moveTurret called with heading = "));
  Serial.print(heading);
  Serial.print(F(", units = "));
  if (units == degrees) {
    Serial.print(F("degrees, rel = "));
  } else {Serial.print(F("counts, rel = ")); }
  Serial.println(relativeToRobot);

  if (enabled) {
    // todo
    if (relativeToRobot) {
      // todo: use encoder + laser to determine position
      // targetRelativeHeading = heading;

      if (units == degrees) {
        // old code that does not work well
        // moveTurret(heading, counts, power, relativeToRobot, ramp);
      } else if (units == counts) {
        int8_t sign = copysign(1, currentTurretEncoderCount);
        // currentTurretEncoderCount = abs(currentTurretEncoderCount);
        // currentTurretEncoderCount %= 360;
        // currentTurretEncoderCount *= sign;

        targetTurretEncoderCount = (int) round((double) heading * QB_COUNTS_PER_TURRET_DEGREE);
        turretMoving = true;

        // sign now used for target instead of current
        sign = 1; // 1 when positive, -1 when negative

        // if the current turret encoder count is over halfway to a full rotation in either direction
        // if (abs(currentTurretEncoderCount) > (QB_COUNTS_PER_TURRET_REV / 2)) {
        //   // ensure that the target and current counts are as close as possible
        //   // if adding one rotation's worth of counts would help, do so
        //   if (currentTurretEncoderCount - (targetTurretEncoderCount + QB_COUNTS_PER_TURRET_REV) < currentTurretEncoderCount - targetTurretEncoderCount) {
        //     targetTurretEncoderCount += QB_COUNTS_PER_TURRET_REV;
        //   }
        // }

        if (targetTurretEncoderCount < currentTurretEncoderCount) {
          sign = -1;
        }

        setTurretSpeed(QB_HANDOFF/4 * sign); //! temp constant, implement P loop soon
        delay(100);
        setTurretSpeed(QB_HANDOFF/3 * sign); //! temp constant, implement P loop soon
        delay(100);
        setTurretSpeed(QB_HANDOFF/2 * sign); //! temp constant, implement P loop soon
        delay(100);
        setTurretSpeed(QB_HANDOFF * sign); //! temp constant, implement P loop soon

        // currentTurretEncoderCount = targetTurretEncoderCount; // currentTurretEncoderCount is updated by interrupt
      }

      // currentRelativeHeading = targetRelativeHeading;
    } else {
      // relative to field
      // todo: use magnetometer
    }
  } else {
    setTurretSpeed(0);
  }
}

void QuarterbackTurret::moveTurretAndWait(int16_t heading, float power, bool relativeToRobot, bool ramp) {
  moveTurret(heading, power, relativeToRobot, ramp);
  while (turretMoving && !testForDisableOrStop()) {
    updateTurretMotionStatus();
    delay(10);
  }
}
#pragma endregion

void QuarterbackTurret::updateTurretMotionStatus() {
  // if (utmsCtr >= UTMS_CTR_MAX) {
  //   utmsCtr = 0;
    // Serial.print(F("update called with ctec = "));
    // Serial.print(currentTurretEncoderCount);
    // Serial.print(F("; ttec = "));
    // Serial.print(targetTurretEncoderCount);
    // Serial.print(F("; error (ct) = "));
    // Serial.println(fabs((currentTurretEncoderCount % QB_COUNTS_PER_TURRET_REV) - targetTurretEncoderCount));
  // } else {
  //   utmsCtr++;
  // }

  // determines if encoder is within "spec"
  if (turretMoving && fabs((currentTurretEncoderCount % QB_COUNTS_PER_TURRET_REV) - targetTurretEncoderCount) < QB_TURRET_THRESHOLD) {
    turretMoving = false;
    setTurretSpeed(0);
  }
}

// deprecated
void QuarterbackTurret::turretDirectionChanged() {
  if (currentTurretSpeed > 0 && targetTurretSpeed < 0) {
    // going CW, trying to go CCW
    currentTurretEncoderCount -= slopError;
    targetTurretEncoderCount -= slopError;
  } else if (currentTurretSpeed < 0 && targetTurretSpeed > 0) {
    // going CCW, trying to go CW
    currentTurretEncoderCount += slopError;
    targetTurretEncoderCount += slopError;
  }
}

//* get current heading in degrees
int16_t QuarterbackTurret::getCurrentHeading() {
  int h = (int) round((double) currentTurretEncoderCount / QB_COUNTS_PER_TURRET_DEGREE);
  return (int16_t) NormalizeAngle(h);
}

// Function to normalize an angle to the range [0, 360)
int QuarterbackTurret::NormalizeAngle(int angle) {
  while (angle < 0) {
    angle += 360;
  }
  angle %= 360; // Ensure angle is within [0, 360) range
  return angle;
}

// Function to calculate the shortest rotation direction
// Returns -1 for counterclockwise, 1 for clockwise, or 0 if no rotation needed
int QuarterbackTurret::CalculateRotation(float currentAngle, float targetAngle) {
  // Normalize inputs to [0,360)
  int cur = NormalizeAngle((int)round(currentAngle));
  int tgt = NormalizeAngle((int)round(targetAngle));

  int delta = tgt - cur;
  // Normalize to [-180, 180)
  while (delta > 180) delta -= 360;
  while (delta <= -180) delta += 360;

  return delta; // signed shortest rotation in degrees (positive -> rotate CW, negative -> CCW)
}

// not currently used
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
    return (adjustedCurrentHeading - positiveHeading);
  } else {
    // positive heading is closer
    return (adjustedCurrentHeading - negativeHeading);
  }
}

// not currently used
int16_t QuarterbackTurret::findNearestHeading(int16_t targetHeading) {
  return findNearestHeading(targetHeading, currentRelativeHeading);
}

#pragma region Assembly
void QuarterbackTurret::aimAssembly(AssemblyAngle angle, bool force) {
  if (enabled) {
    if (!assemblyMoving) {
      targetAssemblyAngle = angle;
      if (targetAssemblyAngle != currentAssemblyAngle || force) {
        moveAssemblySubroutine();
      }

      //* force is a blocking routine to ensure it works without interruption
      //* do not use force frequently as it can strain the motor
      //* this should only be used on startup
      if (force) {
        // also allow emergency stop
        while ((millis() - assemblyStartTime) <= QB_ASSEMBLY_TILT_DELAY && !testForDisableOrStop()) {
          NOP();
        }
        currentAssemblyAngle = targetAssemblyAngle;
        assemblyMoving = false;
        assemblyMotor.write(0);
      }
    } else if ((millis() - assemblyStartTime) > QB_ASSEMBLY_TILT_DELAY) {
      currentAssemblyAngle = targetAssemblyAngle;
      assemblyMoving = false;
      assemblyMotor.write(0);
    }
  } else {
    assemblyMotor.write(0);
  }
}

//! this is a dangerous function to call
// should only be called with known good state
void QuarterbackTurret::moveAssemblySubroutine() {
  if (targetAssemblyAngle == straight) {
    assemblyMotor.write(QB_ASM_SPEED);
  } else if (targetAssemblyAngle == angled) {
    assemblyMotor.write(-1.25*QB_ASM_SPEED);
  }
  assemblyStartTime = millis();
  assemblyMoving = true;
}
#pragma endregion

#pragma region Cradle
//! this is a dangerous function to call
// should only be called with known good state
void QuarterbackTurret::moveCradleSubroutine() {
  // Serial.print(F("target neq current | "));
  if (targetCradleState == forward) {
    // move forwards
    cradleActuator.write(1.0);
    cradleStartTime = millis();
    cradleMoving = true;
    // Serial.print(F("cradle moving forward | "));
  } else if (targetCradleState == back) {
    // move backwards
    cradleActuator.write(-1.0);
    cradleStartTime = millis();
    cradleMoving = true;
    // Serial.print(F("cradle moving backward | "));
  }
}

void QuarterbackTurret::moveCradle(CradleState state, bool force) {
  if (enabled) {
    if (!cradleMoving) {
      targetCradleState = state;
      // Serial.print(F("current state: "));
      // if (currentCradleState == forward) {
      //   Serial.print(F("forward | "));
      // } else if (currentCradleState == back) {
      //   Serial.print(F("backward | "));
      // }
      // Serial.print(F("target state: "));
      // if (targetCradleState == forward) {
      //   Serial.print(F("forward | "));
      // } else if (targetCradleState == back) {
      //   Serial.print(F("backward | "));
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
      // Serial.print(F("cradle stopped | "));
    }
  } else {
    cradleActuator.write(0);
  }
  // Serial.println();
}
#pragma endregion

#pragma region Flywheels
void QuarterbackTurret::setFlywheelSpeed(float absoluteSpeed) {
  // update the motors so they are spinning at the new speed
  if (enabled) {
    // if current speed is not the passed speed, change the motor speed. this is only to avoid unnecessary writes
    if (fabs(currentFlywheelSpeed - absoluteSpeed) > 0.01) {
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
  // If in combine, use different preset flywheel speeds, which are set for combine distances
  if(mode == combine){
    targetFlywheelStage = stage;
    setFlywheelSpeed(combineSpeeds[static_cast<int>(targetFlywheelStage)]);
    currentFlywheelStage = targetFlywheelStage;
  } else{
    targetFlywheelStage = stage;
    setFlywheelSpeed(flywheelSpeeds[static_cast<int>(targetFlywheelStage)]);
    currentFlywheelStage = targetFlywheelStage;
  }
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
#pragma endregion

#pragma region Auto Mode
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

void QuarterbackTurret::readTargetingInfo() {
  static boolean recvInProgress = false;
  newData = false;
  static int ndx = 0;

  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial2.available() > 0 && !newData) {
    rc = Serial2.read();

    if (recvInProgress) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= NUM_CHARS - 1) ndx = NUM_CHARS - 1;
      } else {
        receivedChars[ndx] = '\0';
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    } else if (rc == startMarker) {
      recvInProgress = true;
      ndx = 0;
    }
  }

  if (newData) {
    String packet = String(receivedChars);
    packet.trim();

    // Debug: show raw incoming packet
    // Serial.print("RAW PACKET: ");
    // Serial.println(packet);

    // Clean up < > if present
    if (packet.startsWith("<")) packet = packet.substring(1);
    if (packet.endsWith(">")) packet = packet.substring(0, packet.length() - 1);
    packet.trim();

    // Split into tokens by comma
    int commaPos[5] = {-1, -1, -1, -1, -1};
    int count = 0;
    int pos = 0;
    while ((pos = packet.indexOf(',', pos)) != -1 && count < 5) {
      commaPos[count++] = pos;
      pos++;
    }

    // Ensure we have at least type + x + y + z + q (5 commas = 6 parts)
    if (count < 4) return;  // Not enough fields → skip

    String type = packet.substring(0, commaPos[0]);

    // Find x, y (we ignore z and q for now, but parse them correctly)
    double newX = packet.substring(commaPos[0] + 1, commaPos[1]).toFloat();
    double newY = packet.substring(commaPos[1] + 1, commaPos[2]).toFloat();
    // double newZ = packet.substring(commaPos[2] + 1, commaPos[3]).toFloat(); // optional
    // double newQ = packet.substring(commaPos[3] + 1).toFloat();             // optional

    if (type == "QB") {
      if (fabs(newX) > 0.1 && fabs(newY) > 0.1) {
        position[0] = newX;   // X coordinate
        position[1] = newY;   // Y coordinate
        // Serial.print("Updated QB: X=");
        // Serial.print(newX, 2);
        // Serial.print(", Y=");
        // Serial.println(newY, 2);
      }
    } 
    else if (type == "RCV") {
      // Round-robin assignment (or switch to ID-based later)
      static int nextRcvIndex = 0;
      int index = nextRcvIndex % NUM_RECEIVERS;
      nextRcvIndex++;

      if (fabs(newX) > 0.1 && fabs(newY) > 0.1) {
        receivers[index].position[0] = newX;
        receivers[index].position[1] = newY;
        // Serial.print("Updated Receiver ");
        // Serial.print(index);
        // Serial.print(": X=");
        // Serial.print(newX, 2);
        // Serial.print(", Y=");
        // Serial.println(newY, 2);
      }
    }
  }
}

/* @brief Calculates the target angle based on the location of the target receiver relative to the center of the QB
 * @author Kaiden Colish
 * @date 2025-07-21
 */
int QuarterbackTurret::angleToTarget(Receiver receiver){
  double dx = receiver.position[0] - position[0];
  double dy = receiver.position[1] - position[1];

  // Guard: if receiver and QB at same position, return 0
  if (fabs(dx) < 0.01 && fabs(dy) < 0.01) {
    return 0; // Return forward if no offset
  }

  // Standard bearing: 0 at north, clockwise positive
  double ang = atan2(dx, dy) * 180.0 / PI;
  if (ang < 0) ang += 360.0;

  ang += 180.0;  // Add 180 to flip the direction
  if (ang >= 360.0) ang -= 360.0;

  return NormalizeAngle((int)round(ang));
}

float QuarterbackTurret::distanceToTarget(Receiver receiver){
  float distance = sqrt(pow(receiver.position[0] - position[0], 2) + pow(receiver.position[1] - position[1], 2));
  return distance;
}

void QuarterbackTurret::moveToTarget(int targetHeading){
  targetRelativeHeading = targetHeading;
  targetTurretEncoderCount = (int) round((double) targetRelativeHeading * QB_COUNTS_PER_TURRET_DEGREE);
  setTurretSpeed(QB_HOME_MAG * copysign(1, targetRelativeHeading), true);

  while ((currentTurretEncoderCount < targetTurretEncoderCount - QB_TURRET_THRESHOLD || currentTurretEncoderCount > targetTurretEncoderCount + QB_TURRET_THRESHOLD) && !testForDisableOrStop()){
    // Run until turret reaches target position
  }

  setTurretSpeed(0,true);
}

/*
 * @brief Sets the flywheel speed based on the distance (in feet) to the target. Will calculate the distance if it is not provided.
 * @param distance The distance to the target in feet
 * @return The calculated flywheel speed
 * @author Kaiden Colish
 * @date 2025-07-30
 */
float QuarterbackTurret::setAutoFlywheelSpeed(float distance){
  float dist = distance;
  if (dist < 0.01) {
    dist = distanceToTarget(receivers[currReceiver]);
  }

  if(dist < 1){
    setFlywheelSpeed(0);
    return 0;
  }

  dist = dist + 0.30;

  //working okay: 0.0413 + 0.105*dist - 0.0167*pow(dist, 2) + 0.0016*pow(dist, 3);
  //Equation that could work: 0.0413 + 0.105*dist - 0.0167*pow(dist, 2) + 0.0016*pow(dist, 3); old 0.0207 + 0.1368*dist - 0.0281*pow(dist, 2) + 0.0027*pow(dist, 3);
  float speed = 0.0207 + 0.1368*dist - 0.0281*pow(dist, 2) + 0.0027*pow(dist, 3); // https://docs.google.com/spreadsheets/d/1Bzx51mkd1ly9TguSG5dGD3yGMdKhlyRx6Mq69FKj0ZQ/edit?usp=sharing
  setFlywheelSpeed(speed);

  return speed; // Return the speed for debugging purposes (may not be needed)
}
#pragma endregion

#pragma region Macros
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

  int16_t targetHeading = (getCurrentHeading() + 130) % 360;

  calculateHeadingMag();
  targetAbsoluteHeading = headingDeg + 180;
  targetAbsoluteHeading %= 360;

  if (useMagnetometer) {
    //moveTurretAndWait(targetHeading);
    //Use the magnetometer to make sure we get close to the requested angle
    //calculateHeadingMag();
    //holdTurretStill();
    //cradleActuator.write(1.0);
    setFlywheelSpeedStage(slow_outwards);
    long currentTime = millis();
    while ((currentTime + 4000) > millis()) {
      calculateHeadingMag();
      turretPIDSpeed = turretPIDController(headingDeg, (float)targetAbsoluteHeading, .01, 0, 0, .25);
      setTurretSpeed(turretPIDSpeed, true);
    }
    cradleActuator.write(1.0);
    delay(2000);
  } else {
    targetHeading += 10;
    targetHeading %= 360;
    moveTurretAndWait(targetHeading);
    cradleActuator.write(1.0);
    setFlywheelSpeedStage(slow_outwards);
    delay(2000);
    setFlywheelSpeedStage(stopped);
  }

  cradleActuator.write(-1);
  delay(2000);
  cradleActuator.write(0);
  this->runningMacro = false;
}

/**
 * @brief Moves turret to combine right position
 * @author Kaiden Colish
 * @date 2025-06-04
 */
void QuarterbackTurret::combineMoveRight(){
  if (this->combinePosition == combineStraight) {
    this->combinePosition = combineRight;
    // moveTurretAndWait(45);
    targetRelativeHeading = 40;
    targetTurretEncoderCount = (int) round((double) targetRelativeHeading * QB_COUNTS_PER_TURRET_DEGREE);
    setTurretSpeed(QB_HOME_MAG * copysign(1, targetRelativeHeading), true);
    while ((currentTurretEncoderCount < targetTurretEncoderCount - QB_TURRET_THRESHOLD || currentTurretEncoderCount > targetTurretEncoderCount + QB_TURRET_THRESHOLD) && !testForDisableOrStop()){
      // Run until turret reaches target position
    }
    setTurretSpeed(0,true);
  } else if (this->combinePosition == combineLeft) {
    this->combinePosition = combineStraight;
    // moveTurretAndWait(0);
    targetRelativeHeading = 0;
    targetTurretEncoderCount = (int) round((double) targetRelativeHeading * QB_COUNTS_PER_TURRET_DEGREE);
    setTurretSpeed(QB_HOME_MAG * copysign(1, -targetRelativeHeading), true);
    while ((currentTurretEncoderCount < targetTurretEncoderCount - QB_TURRET_THRESHOLD) && !testForDisableOrStop()){
      // Run until turret reaches target position
    }
    setTurretSpeed(0,true);
  }
}

/**
 * @brief Moves turret to combine left position
 * @author Kaiden Colish
 * @date 2025-06-04
 */
void QuarterbackTurret::combineMoveLeft(){
  if (this->combinePosition == combineStraight) {
    this->combinePosition = combineLeft;
    // moveTurretAndWait(-45);
    targetRelativeHeading = -40;
    targetTurretEncoderCount = (int) round((double) targetRelativeHeading * QB_COUNTS_PER_TURRET_DEGREE);
    setTurretSpeed(QB_HOME_MAG * copysign(1, targetRelativeHeading) * 1.5, true);
    while ((currentTurretEncoderCount < targetTurretEncoderCount - QB_TURRET_THRESHOLD || currentTurretEncoderCount > targetTurretEncoderCount + QB_TURRET_THRESHOLD) && !testForDisableOrStop()){
      // Run until turret reaches target position
    }
    setTurretSpeed(0,true);
  } else if (this->combinePosition == combineRight) {
    this->combinePosition = combineStraight;
    // moveTurretAndWait(0);
    targetRelativeHeading = 0;
    targetTurretEncoderCount = (int) round((double) targetRelativeHeading * QB_COUNTS_PER_TURRET_DEGREE);
    setTurretSpeed(QB_HOME_MAG * copysign(1, targetRelativeHeading) * -1.5, true);
    while ((currentTurretEncoderCount < targetTurretEncoderCount - QB_TURRET_THRESHOLD || currentTurretEncoderCount > targetTurretEncoderCount + QB_TURRET_THRESHOLD) && !testForDisableOrStop()){
      // Run until turret reaches target position
    }
    setTurretSpeed(0,true);
  }
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
  calibMagnetometer();
  this->runningMacro = false;
}

void QuarterbackTurret::reset() {
  this->enabled = true;
  this->runningMacro = true;
  moveCradle(back, true); // force
  aimAssembly(straight);
  // loadFromCenter();
  zeroTurret(); // temp: just zero
  this->initialized = true;
  this->runningMacro = false;
}
#pragma endregion

#pragma region Safety
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
  } else {
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
#pragma endregion

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

#pragma region Magnetometer
/**
 * @brief Sets up magnetometer
 * @authors Rhys Davies, Corbin Hibler
 * @date 2024-01-03
 */
void QuarterbackTurret::magnetometerSetup() {
    if (! lis3mdl.begin_I2C()) {          // hardware I2C mode, can pass in address & alt Wire
    //if (! lis3mdl.begin_SPI(LIS3MDL_CS)) {  // hardware SPI mode
    //if (! lis3mdl.begin_SPI(LIS3MDL_CS, LIS3MDL_CLK, LIS3MDL_MISO, LIS3MDL_MOSI)) { // soft SPI
        Serial.println("Failed to find LIS3MDL chip");
    }
    Serial.println("LIS3MDL Found!");

    lis3mdl.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);
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

    lis3mdl.setDataRate(LIS3MDL_DATARATE_1000_HZ);
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

/**
 * @brief Spins the turret 360 degrees slowly to allow magnetometer to calibrate itself on startup
 * @author George Rak
 * @date 4-9-2024
 */
void QuarterbackTurret::calibMagnetometer() {
  mag_yVal = 0;
  mag_xVal = 0;
  mag_xMax = -1000000;
  mag_xMin = 1000000;
  mag_xHalf = 0;
  mag_yMax = -1000000;
  mag_yMin = 1000000;
  mag_yHalf = 0;
  mag_xSign = false;
  mag_ySign = false;
  northHeadingDegrees = 7.0f;

  long startTime = millis();
  setTurretSpeed(QB_HOME_MAG, true);

  while (millis() - startTime < 15000 && !testForDisableOrStop()){
    // get X Y and Z data all at once
    lis3mdl.read();

    //Constantly looking for min and max values of X
    if (lis3mdl.x < mag_xMin && lis3mdl.x != -1 && lis3mdl.x != 0) {mag_xMin = lis3mdl.x;}
    else if (lis3mdl.x > mag_xMax && lis3mdl.x != -1 && lis3mdl.x != 0) {mag_xMax = lis3mdl.x;}

    //Adjusting X values to range from + or - values rather than all positive
    mag_xHalf = abs(mag_xMax) - abs(mag_xMin);
    mag_xHalf/= 2;
    mag_xHalf+= abs(mag_xMin);

    //Constantly looking for min and max values of Y
    if (lis3mdl.y < mag_yMin && lis3mdl.y != -1 && lis3mdl.y != 0 && lis3mdl.y != 10) {mag_yMin = lis3mdl.y;}
    else if (lis3mdl.y > mag_yMax && lis3mdl.y != -1 && lis3mdl.y != 0 && lis3mdl.y != 10) {mag_yMax = lis3mdl.y;}

    //Adjusting Y values to range from + or - values rather than all positive
    mag_yHalf = abs(mag_yMax) - abs(mag_yMin);
    mag_yHalf/= 2;
    mag_yHalf+= abs(mag_yMin);

    /*DEBUGGING PRINTOUTS*/
    //Serial.print("X: "); Serial.print(lis3mdl.x);
    //Serial.print("\tY: "); Serial.print(lis3mdl.y);
    //Serial.print("\tMinX: "); Serial.print(mag_xMin);
    //Serial.print("\tMaxX: "); Serial.print(mag_xMax);
    //Serial.print("\tMinY: "); Serial.print(mag_yMin);
    //Serial.print("\tMaxY: "); Serial.print(mag_yMax);
    //Serial.println();
  }

  setTurretSpeed(0, true);

  //Updating variables that will be used to handle other two possible sign cases for each value
  if ((mag_xMax+mag_xMin) < 0) {
    mag_xSign = true;
  }
  if ((mag_yMax+mag_yMin) < 0) {
    mag_ySign = true;
  }

  // calculateHeadingMag();
  Serial.print(F("Magnetometer reading after calib: "));
  Serial.print(headingDeg);
  Serial.print(F("\tEncoder after calib:"));
  Serial.println(currentTurretEncoderCount);
  // delay(5000);

  currentTurretEncoderCount = 0;
  targetTurretEncoderCount = 0;
  turretMoving = false;
  moveTurretAndWait(0, true); // go to zero of the encoder

  magnetometerCalibrated = true;
  calculateHeadingMag(); // calculate current value of magnetometer (headingDeg)

  Serial.print("Target Abs Heading Before 0: ");
  Serial.print(targetAbsoluteHeading);
  Serial.print("\tNorth Heading Degrees: ");
  this->northHeadingDegrees = 0;
  Serial.print(northHeadingDegrees);
  Serial.println();
  // delay(2000);

  // from here on out, headingDeg and targetAbsoluteHeading are offset by northHeadingDegrees
  // headingDeg = 0;
  // targetAbsoluteHeading = 0;

  Serial.println("Magnetometer has been calibrated!");
  eIntegral = 0;
  previousTime = millis();
  setTurretSpeed(0);
  // delay(5000);
}

/**
 * @brief Uses the data collected at calibration to calculate the current heading relative to magnetic north
 * @author George Rak
 * @date 4-9-2024
 */
void QuarterbackTurret::calculateHeadingMag() {
  //Only run the code in here if the calibration has been done to the magnetometer
  if (magnetometerCalibrated) {
    lis3mdl.read();

    //Calculate the current angle of the turret based on the calibration data
    if (mag_xSign) {
      mag_xVal = lis3mdl.x + mag_xHalf;
    } else {
      mag_xVal = lis3mdl.x - mag_xHalf;
    }

    if (mag_ySign) {
      mag_yVal = lis3mdl.y + mag_yHalf;
    } else {
      mag_yVal = lis3mdl.y - mag_yHalf;
    }

    //Evaluate both ranges of X and Y then scale the smaller value to be within the same range as the larger
    if (mag_yHalf > mag_xHalf) {
      mag_xVal = (double)((double)mag_xVal/((double)mag_xHalf))*(double)mag_yHalf;
    } else if (mag_xHalf > mag_yHalf) {
      mag_yVal = (double)((double)mag_yVal/((double)mag_yHalf))*(double)mag_xHalf;
    }

    //Calculate angle in radians
    if (mag_xVal != -1 && mag_xVal !=0 && mag_yVal != 0 && mag_yVal != -1) {
      headingRad = atan2(mag_yVal, mag_xVal);
    }

    //Convert to degrees
    headingDeg = headingRad*180/M_PI;

    //If the degrees are negative then they just need inversed plus 180
    if (headingDeg < 0) {
      headingDeg += 360;
    }

    // integrate offset into measurement
        headingDeg = fmod(headingDeg + 7.0 + QB_DECLINATION + 360.0, 360.0);   // +7° for Ada, OH
    if (headingDeg > 360) headingDeg = ((int) headingDeg) % 360;

    // EMA for heading (initialize on first read)
    if (headingBufferCount == 0) {
      headingDegSmoothed = headingDeg;
    } else {
      headingDegSmoothed = emaAlpha * headingDeg + (1 - emaAlpha) * headingDegSmoothed;
    }
    headingBufferCount = 1;  // EMA doesn't need count

    headingDeg = headingDegSmoothed;  // Overwrite raw with EMA-smoothed

    /*DEBUGGING PRINTOUTS*/
    // Serial.print("X: "); Serial.print(lis3mdl.x);
    // Serial.print("\tY: "); Serial.print(lis3mdl.y);
    // Serial.print("\tMinX: "); Serial.print(mag_xMin);
    // Serial.print("\tMaxX: "); Serial.print(mag_xMax);
    // Serial.print("\tMinY: "); Serial.print(mag_yMin);
    // Serial.print("\tMaxY: "); Serial.print(mag_yMax);
    // Serial.print("\txAdapt: "); Serial.print(mag_xVal);
    // Serial.print("\tyAdapt: "); Serial.print(mag_yVal);
    // Serial.print("\tHeading [deg]: "); Serial.print(headingDeg);
    // Serial.println();
  }
}

void QuarterbackTurret::holdTurretStill() {
  if (magnetometerCalibrated) {
    float maxSpeed = 0.2f;
    if (motor1Value > 25 || motor2Value > 25) {
      //We should limit the rotation rate of the turret since the base is moving as well and we don't want the robot to flip
      maxSpeed = 0.1f;
    }
    //Run the PID loop
    turretPIDSpeed = turretPIDController(headingDeg, (float)targetAbsoluteHeading, kp, kd, ki, maxSpeed);
    setTurretSpeed(turretPIDSpeed, true);
  }
}

float QuarterbackTurret::turretPIDController(float current, float target, float kp, float kd, float ki, float maxSpeed) {
  if (maxSpeed > .5) {
    maxSpeed = .5;
  } else if (maxSpeed < -.5) {
    maxSpeed = -.5;
  }

  // Measure the time elapsed since last iteration
  long currentTime = millis();
  float deltaT = ((float)(currentTime - previousTime)) / 1000.0f;  // Convert to seconds for proper scaling

  // PID loops should update as fast as possible but if it waits too long this could be a problem
  if (deltaT > QB_TURRET_PID_MIN_DELTA_T / 1000.0f && deltaT < QB_TURRET_PID_MAX_DELTA_T / 1000.0f) {
    // Find which direction will be closer to requested angle
    int e = CalculateRotation(current, target);

    // Taking the average of the error
    prevErrorVals[prevErrorIndex] = e;
    prevErrorIndex++;
    prevErrorIndex %= PID_ERROR_AVG_ARRAY_LENGTH;

    // Taking the average for error (removed the initial fill loop to avoid resetting the array every time)
    int avgError = 0;
    for (int i = 0; i < PID_ERROR_AVG_ARRAY_LENGTH; i++) {
      avgError += prevErrorVals[i];
    }
    avgError /= PID_ERROR_AVG_ARRAY_LENGTH;
    Serial.println("e before:");
    Serial.println(e);

    // Add deadband: If error < 3 degrees, stop motor and zero integral to prevent sway
    if (abs(e - 180) < 3) {
      float u = 0.0f;  // Declare and set to zero here
      eIntegral = 0;   // Prevent windup
      // Optional: Log for debugging (comment out if too spammy)
      Serial.println("Deadband applied: error < 3 deg, u=0");
      Serial.println("e after:");
      Serial.println(abs(e - 180));
      return u;
    }

    // Calculate the derivative and integral values (now scaled by deltaT in seconds)
    float eDerivative = (e - ePrevious) / deltaT;
    eIntegral += e * deltaT;

    // Anti-windup: Clamp integral to prevent excessive buildup
    const float integralLimit = 10.0f;  // Adjust based on testing; prevents windup
    if (eIntegral > integralLimit) eIntegral = integralLimit;
    if (eIntegral < -integralLimit) eIntegral = -integralLimit;

    // Compute the PID control signal
    float u = (kp * e) + (ki * eIntegral) + (kd * eDerivative);

    // Constrain output PWM values to maxSpeed
    if (u > maxSpeed) {
      u = maxSpeed;
    } else if (u < -maxSpeed) {
      u = -maxSpeed;
    }

    // If PWM value is less than the minimum PWM value needed to move the robot,
    if (abs(u) < QB_MIN_PWM_VALUE) {
      u = 0.0f;
    }

    // If the robot gets within an acceptable range then send error etc to 0
    if (abs(e) < QB_TURRET_PID_THRESHOLD) {
      e = 0;
      eDerivative = 0;
      eIntegral = 0;
      ePrevious = 0;
    }

    Serial.print("\tCurrent [deg]: "); Serial.print(current, 0);
    target = (target - 180) + 360;
    if(target >= 360)
    {
      target = target - 360;
    }

    Serial.print("\tTarget [deg]: "); Serial.print(target);
    Serial.print("\tQuarterback:"); Serial.print("x="); Serial.print(position[0]); Serial.print(", y="); Serial.print(position[1]);
    Serial.print("\tReceiver:"); Serial.print("x="); Serial.print(receivers[0].position[0]); Serial.print(", y="); Serial.print(receivers[0].position[1]);
    Serial.println();

    // Update variables for next iteration
    previousTime = currentTime;
    ePrevious = e;

    // Removed asymmetric bias: if(u > 0){ u *= 1.1; } – this can cause directional preference

    // Constrain the values that are sent to the motor while keeping sign
    u = copysign(constrain(abs(u), 0, 1), u); // TODO: maybe not necessary?

    if (e == 0) {
      u = 0.0f;
    }

    return -u;
  } else if (deltaT > QB_TURRET_PID_BAD_DELTA_T / 1000.0f) {
    // Drop the value if the time since last loop is too high so that errors don't spike
    previousTime = currentTime;
    return turretPIDSpeed;
  } else {
    // If the loop runs faster than the minimum time just return the last value and wait for next loop
    return turretPIDSpeed;
  }
}

#pragma endregion

#pragma region Stabilization
/**
 * @brief Reads a UART communication from the other ESP mounted to the turret. This ESP currently provides the speed of both motors on the drivetrain so we know if the robot is moving
 * @author George Rak
 * @date 5-14-2024
 */
// Commented out for UART repurposing
// void QuarterbackTurret::updateReadMotorValues() {
//   recievedMessage = "";

//   //While there are characters available in the buffer read each one individually
//   while (Uart_Turret.available()) {
//     char character = Uart_Turret.read();

//     //Added a delimeter between messages since loop times are different and multiple messages might come in before they are read and the buffer is cleared
//     //Since they are coming so fast and there is no need to remember past values only the most recent is kept
//     if (character == '~') {
//       if (Uart_Turret.available()) {
//         recievedMessage = "";
//       }
//     } else {
//       recievedMessage += character;
//     }
//   }

//   //The Server client relationship between the ESPs knows if they disconnect so it is possible that they might send DISCONNECTED over the communication instead of values, in this case set the value to the max so that the turret spins slower
//   if (recievedMessage!="") {
//     if (recievedMessage == "DISCONNECTED") {
//       motor1Value = 100;
//       motor2Value = 100;
//     } else {
//       //Doing some string formatting here, a delimiter was added between the data to help keep them separate for motor #1 and motor #2
//       motor1Value = (recievedMessage.substring(0, recievedMessage.indexOf('&'))).toInt();
//       motor2Value = (recievedMessage.substring(recievedMessage.indexOf('&') + 1)).toInt();
//     }
//   }

//   // Serial.print("Motor1: ");
//   // Serial.print(motor1Value);
//   // Serial.print("\tMotor2: ");
//   // Serial.print(motor2Value);
//   // Serial.println();
// }
#pragma endregion