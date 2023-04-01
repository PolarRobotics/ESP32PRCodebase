#pragma once

#ifndef _QUARTERBACK_H_
#define _QUARTERBACK_H_

#include <Robot/Robot.h>
#include <Robot/MotorControl.h>

// Flywheel defines 
#define FLYWHEEL_SPEED_FULL 0.3
#define FLYWHEEL_STOP_SPEED 0

#define FW_TIME_INCREMENT 25
#define FW_BRAKE_PERCENTAGE 0.9
#define FW_ACCEL_RATE .025


// Elevation (linear actuators) defines
#define SERVO_SPEED_UP 1
#define SERVO_SPEED_STOP 0
#define SERVO_SPEED_DOWN -1
#define MAX_ELEVATION 100
#define ELEVATION_PERIOD 3750

// Conveyor defines
#define CONVEYOR_ON 1
#define CONVEYOR_OFF 0

// Debounce Vars
#define DEBOUNCE_WAIT 250

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
    int arrayPos = 0;
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
    unsigned long lastFlywheelRampTime;
    float currentFWPower;
    unsigned long lastDBElev = 0, lastDBFW = 0, lastDBFWChange = 0, lastDBConv = 0;
    float flywheelSpeedFactor = 0;
    const float speedFac[4] = {0.0, 0.3, 0.5, 0.8};
  public:
    Quarterback(uint8_t fwpin, 
        uint8_t conveyorpin, uint8_t elevationpin);
    void setup();
    void toggleFlywheels();
    float rampFW(float requestedpwr);
    void aim(qbAim dir);
    void toggleConveyor();
    void changeFWSpeed(speedStatus speed);
    void update();
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
    // Toggle the bool so we know if its on or not
    flywheelsOn = !flywheelsOn;

    lastDBFW = millis();
  }
}

float Quarterback::rampFW(float requestedPower) {

    if (millis() - lastFlywheelRampTime >= FW_TIME_INCREMENT) {
        if (abs(requestedPower) < THRESHOLD) { // if the input is effectively zero
        // Experimental Braking Code
            if (abs(currentFWPower) < 0.1) { // if the current power is very small just set it to zero
                currentFWPower = 0;
            }
            else {
                currentFWPower *= FW_BRAKE_PERCENTAGE;
            }
            lastFlywheelRampTime = millis();
        }
        else if (abs(requestedPower - currentFWPower) < FW_ACCEL_RATE) { // if the input is effectively at the current power
            return requestedPower;
        }
        // if we need to increase speed and we are going forward
        else if (requestedPower > currentFWPower) { 
            currentFWPower = currentFWPower + FW_ACCEL_RATE;
            lastFlywheelRampTime = millis();
        }
        // if we need to decrease speed and we are going forward
        else if (requestedPower < currentFWPower) { 
            currentFWPower = currentFWPower - FW_ACCEL_RATE;
            lastFlywheelRampTime = millis();
        }
    }

    return currentFWPower;
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

void Quarterback::update() { 
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

  // update flywheels if needed
  if (flywheelsOn){
    FWMotor.write(rampFW(FLYWHEEL_SPEED_FULL + flywheelSpeedFactor));
  } else {
    FWMotor.write(rampFW(FLYWHEEL_STOP_SPEED));
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
    // Change the speed factor based on whether the user wants to increase or decrease (HARD CODED)
    switch(speed) {
      case increase: arrayPos++; break;
      case decrease: arrayPos--; break;
    }

    // Cap the arrayPos so it doesn't go out of bounds
    arrayPos = constrain(arrayPos, 0, 3);

    // Set the flywheelSpeedFactor
    flywheelSpeedFactor = speedFac[arrayPos];


    // Update the motors if they are spinning for the new speed
    // if (flywheelsOn){
    //   FWMotor.write(FLYWHEEL_SPEED_FULL + flywheelSpeedFactor);
    // } else {
    //   FWMotor.write(FLYWHEEL_STOP_SPEED);
    // }

    lastDBFWChange = millis();
  }
}

#endif
