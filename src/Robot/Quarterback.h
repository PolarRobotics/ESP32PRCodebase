#pragma once

#ifndef _QUARTERBACK_H_
#define _QUARTERBACK_H_

#include <Robot/Robot.h>
#include <Robot/MotorControl.h>

// Flywheel defines 
#define FLYWHEEL_SPEED_FULL 0.5 // this should be between 90 and 140. 
#define FLYWHEEL_STOP_SPEED 0

// Elevation (linear actuators) defines
#define SERVO_SPEED_UP 1
#define SERVO_SPEED_STOP 0
#define SERVO_SPEED_DOWN -1
#define MAX_ELEVATION 100
#define ELEVATION_PERIOD 3750

// Conveyor defines
#define CONVEYOR_ON 0.5
#define CONVEYOR_OFF 0

// Debounce Vars
#define DEBOUNCE_WAIT 500

// Enum for Increasing or Decreasing Flywheel Speed
enum speedStatus {
  increase, decrease
};

// Enum for whether to aim up or aim down
enum qbAim {
  aimUp, aimDown
};

/**
 * @brief Quarterback Subclass Header
 * @authors Rhys Davies
 */
class Quarterback { //: public Robot
  private: 
    uint8_t m_FlywheelPin;
    uint8_t m_conveyorPin;
    uint8_t m_ElevationPin;
    MotorControl FWMotor;
    MotorControl conveyorMotor;
    MotorControl elevationMotors;
    bool flywheelsOn, conveyorOn;
    bool aimingUp, aimingDown;
    bool raise, lower;
    bool setupMotors;
    uint8_t currentElevation, targetElevation;
    unsigned long lastElevationTime;
    unsigned long lastFlywheelToggleTime;
    unsigned long lastDBElev = 0, lastDBFW = 0, lastDBFWChange = 0, lastDBConv = 0;
    float flywheelSpeedFactor;
  public:
    Quarterback(uint8_t fwpin, 
        uint8_t conveyorpin, uint8_t elevationpin);
    void setup();
    void toggleFlywheels();
    void aim(qbAim dir);
    void toggleConveyor();
    void changeFWSpeed(speedStatus speed);
    void updateAim();
};

Quarterback::Quarterback(uint8_t fwpin, 
        uint8_t conveyorpin, uint8_t elevationpin) {
    // Declare that the flywheels are off
    flywheelsOn = false;

    // Declare the the conveyor is off
    conveyorOn = false;

    // Declare that we are not trying to aim up or down and we are at the current elevation of 0
    aimingUp = false;
    aimingDown = false;
    currentElevation = 0;

    // Label the pins inside the class
    this->m_FlywheelPin = fwpin;
    this->m_conveyorPin = conveyorpin;
    this->m_ElevationPin = elevationpin;
}

void Quarterback::setup() {
    // Attach the motors inside the class to their respective pins
    FWMotor.attach(m_FlywheelPin);
    conveyorMotor.attach(m_conveyorPin);
    elevationMotors.attach(m_ElevationPin);

    // Set the motor to zero so it doesnt spin on startup
    conveyorMotor.write(CONVEYOR_OFF);

    // Lower the Linear Actuators at start up so they are in the bottom position
    // through the updateAim() function
    setupMotors = true;
    lastElevationTime = millis();
}


void Quarterback::toggleFlywheels() {
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

// Aiming related functions
void Quarterback::aim(qbAim dir) {
  // Debounce for button press
  if (millis() - lastDBElev >= DEBOUNCE_WAIT) {
    // Check which direction the user wants and turn the other direction off
    switch(dir) {
      case aimUp: aimingUp = true; aimingDown = false; break;
      case aimDown: aimingDown = true; aimingUp = false; break;
    }

    lastDBElev = millis();
  }
}

  void Quarterback::updateAim() { 
    // Setup the motors on startup so they go down to absolute zero
    if (setupMotors){
      if (millis() - lastElevationTime >= 8000) {
        setupMotors = false;
      } else {
        elevationMotors.write(SERVO_SPEED_DOWN);
      }
    // Control the motors based on if they want to aim up or down
    } else if (millis() - lastElevationTime >= ELEVATION_PERIOD) {
      if (aimingUp && currentElevation < MAX_ELEVATION) {
        // Set the elevation of the top to 50% higher
        elevationMotors.write(SERVO_SPEED_UP);
        currentElevation += 50;
        aimingUp = false;
        lastElevationTime = millis();

      } else if (aimingDown && currentElevation > 0) {
        // Set the elevation of the bottom to 50% lower
        elevationMotors.write(SERVO_SPEED_DOWN);
        currentElevation -= 50;
        aimingDown = false;
        lastElevationTime = millis();

      } else {
        // Stop the linear actuators if it is not supposed to move up or down anymore
        elevationMotors.write(SERVO_SPEED_STOP);
      }
    }
}

void Quarterback::toggleConveyor() {
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

void Quarterback::changeFWSpeed(speedStatus speed) {
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
