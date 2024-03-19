#pragma once

#ifndef MECANUM_CENTER_H
#define MECANUM_CENTER_H

#include <Robot/Robot.h>
#include <Robot/MotorControl.h>
#include <ps5Controller.h> // ESP PS5 library, access using global instance `ps5`

// TODO: we should seriously reconsider the use of defines, since the names are duplicated between QB and MC
// Flywheel defines 
#define MC_FLYWHEEL_SPEED_FULL 0.5
#define MC_FLYWHEEL_STOP_SPEED 0

// Conveyor defines
#define MC_CONVEYOR_ON 0.5
#define MC_CONVEYOR_OFF 0

// Debounce Vars
#define MC_DEBOUNCE_WAIT 500

/**
 * @brief Mecanum Center Subclass Header
 * @authors Rhys Davies, Andrew Nelson
 */
class MecanumCenter : public Robot {
  private: 
    uint8_t flywheelPin;
    uint8_t conveyorPin;
    MotorControl flywheelMotor;
    MotorControl conveyorMotor;
    bool flywheelsOn, conveyorOn;
    uint8_t currentElevation, targetElevation;
    unsigned long lastFlywheelToggleTime;
    unsigned long lastDBFW = 0, lastDBFWChange = 0, lastDBConv = 0;
    float flywheelSpeedFactor;
  public:
    MecanumCenter(uint8_t fwpin, uint8_t conveyorpin);
    void action() override; //! robot subclass must override action
    void toggleFlywheels();
    void toggleConveyor();
    void changeFWSpeed(SpeedStatus speed);
    void debug();
};

#endif // MECANUM_CENTER_H
