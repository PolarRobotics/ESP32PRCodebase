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
    // motor pins
    uint8_t assemblyPin; // not sure if this is needed
    uint8_t cradlePin;
    uint8_t flywheelPin;
    uint8_t turretPin;

    // motor instances
    MotorControl cradleMotor;
    MotorControl flywheelMotor;
    MotorControl turretMotor;

    // state
    bool enabled = false; // default false, set to true upon homing/reset

    // assembly
    AssemblyAngle currentAssemblyAngle; // initial state unknown, will reset to straight
    AssemblyAngle targetAssemblyAngle; // default straight
    bool assemblyMoving; // default false

    // cradle
    CradleState currentCradleState; // initial state unknown, will reset to back
    CradleState targetCradleState; // default back
    bool cradleMoving; // default false

    // mode
    TurretMode mode; // default manual
    TargetReceiver target; // default receiver_1

    // flywheel
    FlywheelSpeed currentFlywheelStage; // default stopped
    FlywheelSpeed targetFlywheelStage;  // default stopped

    float currentFlywheelSpeed; // default 0
    float targetFlywheelSpeed;  // default 0

    bool flywheelManualOverride; // default false, true when stick controlling flywheel

    // turret
    float currentTurretSpeed; // default 0
    float targetTurretSpeed;  // default 0

    int currentTurretHeading; // default 0
    int targetTurretHeading;  // default 0
    
  public:
    QuarterbackTurret(
      uint8_t flywheelPin,
      uint8_t turretPin,
      uint8_t cradlePin
    );

    void action() override; //! robot subclass must override action

    //* base/internal functions
    void setTurretSpeed(float absoluteSpeed); // moves turret at specified speed (open loop)

    void moveTurret(int heading); // moves turret/turntable to specific heading. currently relative to robot, not field.
                                  // will not be implemented until MotorControl is stabilized

    void aimAssembly(AssemblyAngle angle); // aims the assembly holding the flywheels and cradle (two states).

    void moveCradle(CradleState state); // moves the linear actuator connected to the cradle to one of two states.
    // used to fire the ball in both manual and automatic modes

    void setFlywheelSpeed(double absoluteSpeed); // also accessible via manual stick override

    void setFlywheelSpeedStage(FlywheelSpeed stage); // directly set flywheel speed to a stage

    //* derived functions (general)
    void adjustFlywheelSpeedStage(SpeedStatus speed);

    //* derived functions (automatic targeting)
    void switchMode(TurretMode mode); // switch between manual and automatic targeting
    void switchTarget(TargetReceiver target); // switch between targeted receivers. switches to automatic targeting if not already set.

    //* derived functions (manual macros)
    void loadFromCenter(); // prepares qb to intake ball from center
    void handoff(); // hands off ball to runningback

    //* setup and safety functions
    void setEnabled(bool enabled); // toggles turret and flywheel movement
    void emergencyStop();
    void zeroTurret(); // calibrates turret/moves turret to home/zero (cnc/3d printer style)
    void reset(); // zero turret, aim down (straight), and set flywheels to slow intake
    
};

#endif // QUARTERBACK_TURRET_H
