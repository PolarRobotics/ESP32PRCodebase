#pragma once

#ifndef QUARTERBACK_TURRET_H
#define QUARTERBACK_TURRET_H

#include <Robot/Robot.h>
#include <Robot/MotorControl.h>
#include <ps5Controller.h> // ESP PS5 library, access using global instance `ps5`

enum TurretMode {
  manual, automatic
};

enum TurretAngle {
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

// const float flywheelSpeeds[QB_TURRET_NUM_SPEEDS] = {
//   0,
  
// };


/**
 * @brief Quarterback Turret Subclass Header
 * @authors Maxwell Phillips
 */
class QuarterbackTurret : public Robot {
  private: 
    uint8_t flywheelPin;
    uint8_t turretPin;
    MotorControl flywheelMotor;
    MotorControl turretMotor;
    bool flywheelsOn;
    float flywheelSpeedFactor;
  public:
    QuarterbackTurret();
    void action() override; //! robot subclass must override action

    //* base/internal functions
    void moveTurret(int heading); // moves turret/turntable to specific heading. currently relative to robot, not field.

    void setFlywheelSpeed(double absoluteSpeed); // also accessible via manual stick override

    void setFlywheelSpeedStage(FlywheelSpeed stage); // directly set flywheel speed to a stage

    void aimAssembly(TurretAngle angle); // aims the assembly holding the flywheels and cradle (two states).

    void moveCradle(CradleState state); // moves the linear actuator connected to the cradle to one of two states.
    // used to fire the ball in both manual and automatic modes

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
