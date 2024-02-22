#pragma once

#ifndef QUARTERBACK_TURRET_H
#define QUARTERBACK_TURRET_H

#include <Robot/Robot.h>
#include <Robot/MotorControl.h>
#include <ps5Controller.h> // ESP PS5 library, access using global instance `ps5`

enum TurretMode {
  manual, automatic
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

const float flywheelSpeeds[QB_TURRET_NUM_SPEEDS] = {-0.1, 0, 0.1, 0.3, 0.5, 0.7, 1};

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
    int currentRelativeHeading; // default 0
    int targetRelativeHeading;  // default 0

    // world-relative headings
    int currentAbsoluteHeading; // default 0
    int targetAbsoluteHeading;  // default 0

    uint8_t turretLaserState;
    
  public:
    QuarterbackTurret(
      uint8_t assemblyPin,
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
    void setTurretSpeed(float absoluteSpeed); 

    // moves turret/turntable to specific heading. currently relative to robot, not field.
    // will not be implemented until MotorControl is stabilized
    void moveTurret(int heading, bool relativeToRobot = false); 
      
    // aims the assembly holding the flywheels and cradle (two states).
    void aimAssembly(AssemblyAngle angle); 

    // moves the linear actuator connected to the cradle to one of two states.
    // used to fire the ball in both manual and automatic modes
    void moveCradle(CradleState state); 
    
    // also accessible via manual stick override
    void setFlywheelSpeed(double absoluteSpeed); 

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

    //* setup and safety functions
    void setEnabled(bool enabled); // toggles turret and flywheel movement
    void emergencyStop();
    void zeroTurret(); // calibrates turret/moves turret to home/zero (cnc/3d printer style)
    void reset(); // zero turret, aim down (straight), and set flywheels to slow intake
    
    void printDebug();
};

#endif // QUARTERBACK_TURRET_H
