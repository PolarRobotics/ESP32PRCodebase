#pragma once

#ifndef _MECANUM_CENTER_H_
#define _MECANUM_CENTER_H_

#include <Robot/Robot.h>
#include <Robot/MotorControl.h>

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
    uint8_t m_FlywheelPin;
    uint8_t m_conveyorPin;
    MotorControl FWMotor;
    MotorControl conveyorMotor;
    bool flywheelsOn, conveyorOn;
    uint8_t currentElevation, targetElevation;
    unsigned long lastFlywheelToggleTime;
    unsigned long lastDBFW = 0, lastDBFWChange = 0, lastDBConv = 0;
    float flywheelSpeedFactor;
  public:
    MecanumCenter(uint8_t fwpin, uint8_t conveyorpin);
    void action() override; //! robot subclass must override action
    void setup(); // TODO: merge with constructor
    void toggleFlywheels();
    void toggleConveyor();
    void changeFWSpeed(speedStatus speed);
};

MecanumCenter::MecanumCenter(uint8_t fwpin, uint8_t conveyorpin) {
  // Declare that the flywheels are off
  flywheelsOn = false;

  // Declare the the conveyor is off
  conveyorOn = false;

  // Label the pins inside the class
  this->m_FlywheelPin = fwpin;
  this->m_conveyorPin = conveyorpin;
}

void MecanumCenter::setup() {
  // Attach the motors inside the class to their respective pins
  FWMotor.attach(m_FlywheelPin);
  conveyorMotor.attach(m_conveyorPin);

  // Set the motor to zero so it doesnt spin on startup
  conveyorMotor.write(MC_CONVEYOR_OFF);
}

void MecanumCenter::action() {
  // Toggle the Conveyor and Flywheels
  if (ps5.Square())
    toggleConveyor();
  else if (ps5.Circle())
    toggleFlywheels();
  
  // Change the flywheel speed
  if(ps5.Triangle())
    changeFWSpeed(speedStatus::increase);
  else if (ps5.Cross())
    changeFWSpeed(speedStatus::decrease);
}


void MecanumCenter::toggleFlywheels() {
  if (millis() - lastDBFW >= MC_DEBOUNCE_WAIT) {
    // Toggle the flywheels and use the speed factor to know what speed
    if (!flywheelsOn){
      FWMotor.write(MC_FLYWHEEL_SPEED_FULL + flywheelSpeedFactor);
    } else {
      FWMotor.write(MC_FLYWHEEL_STOP_SPEED);
    }
    // Toggle the bool so we know if its on or not
    flywheelsOn = !flywheelsOn;

    lastDBFW = millis();
  }
}

void MecanumCenter::toggleConveyor() {
  // Debounce for button press
  if (millis() - lastDBConv >= MC_DEBOUNCE_WAIT) {
    // Toggle the conveyor between on or off
    if (!conveyorOn){
      conveyorMotor.write(MC_CONVEYOR_ON);
    } else {
      conveyorMotor.write(MC_CONVEYOR_OFF);
    }
    // Toggle the bool so we know which mode it is in
    conveyorOn = !conveyorOn;
    lastDBConv = millis();
  }
}

void MecanumCenter::changeFWSpeed(speedStatus speed) {
  // Debounce for button press
  if (millis() - lastDBFWChange >= MC_DEBOUNCE_WAIT) {
    // Change the speed factor based on whether the user wants to increase or decrease
    switch(speed) {
      case increase: flywheelSpeedFactor += 0.05; break;
      case decrease: flywheelSpeedFactor -= 0.05; break;
    }
    // Cap it so they only have two levels to speed up and two levels to slow down
    flywheelSpeedFactor = constrain(flywheelSpeedFactor, -0.15, 0.15);

    // Update the motors if they are spinning for the new speed
    if (flywheelsOn){
      FWMotor.write(MC_FLYWHEEL_SPEED_FULL + flywheelSpeedFactor);
    } else {
      FWMotor.write(MC_FLYWHEEL_STOP_SPEED);
    }

    lastDBFWChange = millis();
  }
}

#endif
