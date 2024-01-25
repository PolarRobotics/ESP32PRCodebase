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
    void toggleFlywheels();
    void changeFlywheelSpeed(SpeedStatus speed);
    void switchMode(TurretMode mode);
    void switchReceiver();
    void changeAngle(TurretAngle angle);
    void moveTurret(int heading);
    void moveCradle(CradleState state);
    void receiveBall();
    void launchBall();
    void passToCenter();
    void passToSelectedReceiver();
};

#endif // QUARTERBACK_TURRET_H
