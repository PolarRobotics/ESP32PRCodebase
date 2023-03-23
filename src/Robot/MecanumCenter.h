#pragma once

#ifndef _MECANUM_CENTER_H_
#define _MECANUM_CENTER_H_

#include <Robot/Robot.h>
#include <Robot/MotorControl.h>

// Flywheel defines 
#define FLYWHEEL_SPEED_FULL 0.5 // this should be between 90 and 140. 
#define FLYWHEEL_STOP_SPEED 0

// Conveyor defines
#define CONVEYOR_ON 0.5
#define CONVEYOR_OFF 0

// Debounce Vars
#define DEBOUNCE_WAIT 500

// Enum for Increasing or Decreasing Flywheel Speed
enum speedStatus {
  increase, decrease
};

/**
 * @brief Mecanum Center Subclass Header
 * @authors Rhys Davies, Andrew Nelson
 */
class MecanumCenter { //: public Robot
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
    void setup();
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
    conveyorMotor.write(CONVEYOR_OFF);
}


void MecanumCenter::toggleFlywheels() {
  if (millis() - lastDBFW >= DEBOUNCE_WAIT) {
    // Toggle the flywheels and use the speed factor to know what speed
    if (!flywheelsOn){
      FWMotor.write(FLYWHEEL_SPEED_FULL + flywheelSpeedFactor);
    } else {
      FWMotor.write(FLYWHEEL_STOP_SPEED);
    }
    // Toggle the bool so we know if its on or not
    flywheelsOn = !flywheelsOn;

    lastDBFW = millis();
  }
}

void MecanumCenter::toggleConveyor() {
  // Debounce for button press
  if(millis() - lastDBConv >= DEBOUNCE_WAIT) {
    // Toggle the conveyor between on or off
    if (!conveyorOn){
      conveyorMotor.write(CONVEYOR_ON);
    } else {
      conveyorMotor.write(CONVEYOR_OFF);
    }
    // Toggle the bool so we know which mode it is in
    conveyorOn = !conveyorOn;
    lastDBConv = millis();
  }
}

void MecanumCenter::changeFWSpeed(speedStatus speed) {
  // Debounce for button press
  if (millis() - lastDBFWChange >= DEBOUNCE_WAIT) {
    // Change the speed factor based on whether the user wants to increase or decrease
    switch(speed) {
      case increase: flywheelSpeedFactor += 0.05; break;
      case decrease: flywheelSpeedFactor -= 0.05; break;
    }
    // Cap it so they only have two levels to speed up and two levels to slow down
    flywheelSpeedFactor = constrain(flywheelSpeedFactor, -0.15, 0.15);

    // Update the motors if they are spinning for the new speed
    if (flywheelsOn){
      FWMotor.write(FLYWHEEL_SPEED_FULL + flywheelSpeedFactor);
    } else {
      FWMotor.write(FLYWHEEL_STOP_SPEED);
    }

    lastDBFWChange = millis();
  }
}

#endif
