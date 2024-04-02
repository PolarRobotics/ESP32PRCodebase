#pragma once

#ifndef QUARTERBACK_TURRET_H
#define QUARTERBACK_TURRET_H

#include <Robot/Robot.h>
#include <Robot/MotorControl.h>
#include <ps5Controller.h> // ESP PS5 library, access using global instance `ps5`
#include <Utilities/Debouncer.h>

enum TurretMode {
  manual, automatic
};

enum TurretUnits {
  degrees, counts
};

enum AssemblyAngle {
  straight, angled
};

enum CradleState {
  forward, back
};

enum TargetReceiver {
  receiver_1, receiver_2
};

#define QB_TURRET_NUM_SPEEDS 7

enum FlywheelSpeed {
  slow_inwards, stopped, slow_outwards, lvl1_outwards, lvl2_outwards, lvl3_outwards, maximum
};

const float flywheelSpeeds[QB_TURRET_NUM_SPEEDS] = {-0.1, 0, 0.1, 0.3, 0.5, 0.7, 1.0};

#define QB_BASE_DEBOUNCE_DELAY 50L // 50 millseconds for button debouncing by default

#define QB_CRADLE_TRAVEL_DELAY 750L // ~0.75 seconds to fully extend or compress linear actuator

#define QB_CIRCLE_HOLD_DELAY 750L
#define QB_TRIANGLE_HOLD_DELAY 500L
#define QB_CROSS_HOLD_DELAY 500L

#define QB_TURRET_INTERPOLATION_DELAY 5L
#define QB_TURRET_THRESHOLD 35
#define QB_TURRET_STICK_SCALE_FACTOR 0.5

#define QB_HOME_PCT 0.2

//* Question: Why are there so many #defines commented out?
//* Answer: Documentation purposes.

//* Question: Why not just use the #defines as they are written to generate the values?
//* Answer: I don't trust the preprocessor. -MP

#define QB_COUNTS_PER_ENCODER_REV 1000
// 27:1 from falcon to output (1 rev of falcon is 1/27th of turret)
// 5:1 from falcon to encoder gear (12t on falcon, 60t on encoder)
// big gear on encoder needs to spin 5.4 (27/5) times for the turret to make one revolution
// 1 revolution on big gear is the same as 1 revolution on the encoder
// so 5:27 is encoder:turret, and:
// 5/27ths of a revolution of the turret for 1 revolution of the encoder

// #define QB_FALCON_TO_TURRET_RATIO 27 / 1
// #define QB_ENCODER_TO_FALCON_RATIO 5 / 1
// #define QB_ENCODER_TO_TURRET_RATIO /* = */ QB_FALCON_TO_TURRET_RATIO / QB_ENCODER_TO_FALCON_RATIO // (27 / 5) = 5.4
// #define QB_COUNTS_PER_TURRET_REV /* = */ QB_ENCODER_TO_TURRET_RATIO * QB_COUNTS_PER_ENCODER_REV // 5400

#define QB_COUNTS_PER_TURRET_REV /* = */ 5400

// #define QB_COUNTS_PER_TURRET_DEGREE /* = */ QB_COUNTS_PER_TURRET_REV / 360
#define QB_COUNTS_PER_TURRET_DEGREE 15

// there is a large amount of mechanical slop when changing turret directions due to the large gear reduction
// it takes about 10 gear teeth's worth of movement of the encoder gear before the turret actually begins moving again
// for 1000 counts per encoder rev => 5400 counts per turret rev, 10 / 100 gear teeth * 5400 counts / turret = 540 counts

// #define QB_TURRET_SLOP_GEAR_TEETH /* = */ 10 // slop is about 10 gear teeth
// #define QB_TURRET_SLOP_PCT        /* = */ QB_DIRECTION_CHANGE_SLOP_GEAR_TEETH / 100
// #define QB_TURRET_SLOP_COUNTS     /* = */ QB_DIRECTION_CHANGE_SLOP_PCT * QB_COUNTS_PER_TURRET_REV
#define QB_TURRET_SLOP_COUNTS 540

#define QB_TURRET_STOP_LOOP_DELAY_MS 10
#define QB_TURRET_STOP_THRESHOLD_MS 500 // must be a multiple of QB_TURRET_STOP_LOOP_DELAY_MS

#define QB_TURRET_HOME_STOP_FACTOR 0.5 // correction constant for homing, multiplied into stop counts in final homing

//* Enable or Disable Auto Mode for testing
#define QB_AUTO_ENABLED false

/**
 * @brief Quarterback Turret Subclass Header
 * @authors Maxwell Phillips
 */
class QuarterbackTurret : public Robot {
  private: 

    //* motor instances
    MotorControl cradleActuator;
    MotorControl turretMotor;
    MotorControl flywheelLeftMotor;
    MotorControl flywheelRightMotor;
    
    //* pins that must be persistent
    static uint8_t turretEncoderPinA;
    static uint8_t turretEncoderPinB;
    uint8_t turretLaserPin;

    //* joystick inputs
    float stickTurret;   // used to normalize stick input from [0, 255] to [-1.0, 1.0]
    float stickFlywheel; // same as above

    // mode
    TurretMode mode; // default manual
    TargetReceiver target; // default receiver_1

    //* state
    // default false, set to true upon homing/reset, but toggleable via function
    bool enabled; 

    // default false, set to true upon homing/reset. once true, stays true
    // determines whether or not to initiate entire startup/reset routine or just zero the turret
    bool initialized; 

    // default false, set to true while a macro (loadFromCenter or handoff) is running
    bool runningMacro;

    //* assembly
    AssemblyAngle currentAssemblyAngle; // initial state unknown, will reset to straight
    AssemblyAngle targetAssemblyAngle; // default straight
    bool assemblyMoving; // default false

    //* cradle
    CradleState currentCradleState; // initial state unknown, will reset to back
    CradleState targetCradleState; // default back
    bool cradleMoving; // default false
    uint32_t cradleStartTime;


    //* flywheel
    FlywheelSpeed currentFlywheelStage; // default stopped
    FlywheelSpeed targetFlywheelStage;  // default stopped

    float currentFlywheelSpeed; // default 0
    float targetFlywheelSpeed;  // default 0

    bool flywheelManualOverride; // default false, true when stick controlling flywheel

    //* turret
    float currentTurretSpeed; // default 0
    float targetTurretSpeed;  // default 0

    // robot-relative headings
    int16_t currentRelativeHeading; // default undefined
    int16_t targetRelativeHeading;  // default 0

    int32_t currentRelativeTurretCount; // default undefined
    int32_t targetTurretEncoderCount; // default 0
    int32_t errorEncoderCount; // error, i.e. |current - target|
    int32_t slopError; // error due to mechanical slop, automatically determined by robot during reset
    int32_t stopError; // number of encoder counts that are needed for the motor to stop

    bool turretMoving;

    // world-relative headings
    int currentAbsoluteHeading; // default 0
    int targetAbsoluteHeading;  // default 0

    uint8_t turretLaserState;

    // debouncers
    Debouncer* dbOptions;
    Debouncer* dbSquare;
    Debouncer* dbDpadUp;
    Debouncer* dbDpadDown;

    // debouncers for delay
    Debouncer* dbCircle;
    Debouncer* dbTriangle;
    Debouncer* dbCross;
    Debouncer* dbTurretInterpolator;

    // private helper function to avoid code duplication between force and normal case
    void moveCradleSubroutine();

    // moves turret/turntable to specific heading. currently relative to robot, not field.
    //* private helper function
    void moveTurret(int heading, TurretUnits units, bool relativeToRobot = true); 

    //* private helper function to allow managing turret movement asynchronously, and stop it when it reaches the target position
    void updateTurretMotionStatus();
    
  public:
    QuarterbackTurret(
      uint8_t assemblyStepPin,
      uint8_t assemblyDirPin,
      uint8_t cradlePin,
      uint8_t turretPin,
      uint8_t flywheelLeftPin,
      uint8_t flywheelRightPin,
      uint8_t turretEncoderPinA,
      uint8_t turretEncoderPinB,
      uint8_t turretLaserPin
    );

    void action() override; //! robot subclass must override action

    //* base/internal functions
    // moves turret at specified speed (open loop)
    void setTurretSpeed(float absoluteSpeed, bool overrideEncoderTare = false); 

    // moves turret/turntable to specific heading. currently relative to robot, not field.
    void moveTurret(int heading, bool relativeToRobot = true); 

    // moves turret and loops/waits until heading is reached (BLOCKING/SYNCHRONOUS)
    void moveTurretAndWait(int heading, bool relativeToRobot = true);
      
    // aims the assembly holding the flywheels and cradle (two states).
    void aimAssembly(AssemblyAngle angle); 

    // moves the linear actuator connected to the cradle to one of two states.
    // used to fire the ball in both manual and automatic modes
    void moveCradle(CradleState state, bool force = false); 
    
    // also accessible via manual stick override
    void setFlywheelSpeed(float absoluteSpeed); 

    // directly set flywheel speed to a stage
    void setFlywheelSpeedStage(FlywheelSpeed stage); 

    //* derived functions (general)
    void adjustFlywheelSpeedStage(SpeedStatus speed);

    //* derived functions (automatic targeting)
    // switch between manual and automatic targeting
    void switchMode(TurretMode mode); 

    // same as above but toggles automatically instead of explicitly
    void switchMode(); 

    // switch between targeted receivers. switches to automatic targeting if not already set.
    void switchTarget(TargetReceiver target); 

    //* derived functions (manual macros)
    void loadFromCenter(); // prepares qb to intake ball from center
    void handoff(); // hands off ball to runningback
    void testRoutine();

    //* setup and safety functions
    void zeroTurret(); // calibrates turret/moves turret to home/zero (cnc/3d printer style)
    void reset(); // zero turret, aim down (straight), and set flywheels to slow intake
    void setEnabled(bool enabled); // toggles turret and flywheel movement
    void emergencyStop();
    bool testForDisableOrStop();
    
    void printDebug();

    // * encoder
    static int32_t currentTurretEncoderCount;
    static uint8_t turretEncoderStateB; // A channel will be 1 when interrupt triggers
    static void turretEncoderISR();

    void turretDirectionChanged();
};

#endif // QUARTERBACK_TURRET_H
