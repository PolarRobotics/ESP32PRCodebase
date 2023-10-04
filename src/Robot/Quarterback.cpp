#include "Quarterback.h"

Quarterback::Quarterback(
  uint8_t flywheelPin, 
  uint8_t conveyorPin, 
  uint8_t elevationPin
) {
    // The conveyor and flywheels are assumed to be off initially
    flywheelsOn = false;
    conveyorOn = false;

    // Initially we are not aiming up or down, and we assume the initial elevation is zero
    aimingUp = false;
    aimingDown = false;
    currentElevation = 0;

    // Set internal class fields from arguments
    this->flywheelPin = flywheelPin;
    this->conveyorPin = conveyorPin;
    this->elevationPin = elevationPin;

    // Attach the motors to their respective pins
    flywheelMotor.setup(flywheelPin);
    conveyorMotor.setup(conveyorPin);
    elevationMotors.setup(elevationPin);
  
    // Set the conveyor motor to zero so it doesnt spin on startup
    conveyorMotor.write(CONVEYOR_OFF);

    // Lower the Linear Actuators at start up so they are in the bottom position
    // through the updateAim() function
    setupMotors = true;
    lastElevationTime = millis();
}

void Quarterback::action() {
  // Update the bools within the class to see if the user wants to go up or down
  if (ps5.Up())
    aim(QBAim::AIM_UP);
  else if (ps5.Down())
    aim(QBAim::AIM_DOWN);
  
  // Toogle the Conveyor and Flywheels
  if (ps5.Square())
    toggleConveyor();
  else if (ps5.Circle())
    toggleFlywheels();
  
  // Change the flywheel speed
  if(ps5.Triangle())
    changeFWSpeed(SpeedStatus::INCREASE);
  else if (ps5.Cross())
    changeFWSpeed(SpeedStatus::DECREASE);
  
  // Update the aim and flywheels on quarterback to see if we need to stop or not
  update();
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
            if (abs(currentFlywheelPower) < 0.1) { // if the current power is very small just set it to zero
                currentFlywheelPower = 0;
            }
            else {
                currentFlywheelPower *= FW_BRAKE_PERCENTAGE;
            }
            lastFlywheelRampTime = millis();
        }
        else if (abs(requestedPower - currentFlywheelPower) < FW_ACCEL_RATE) { // if the input is effectively at the current power
            return requestedPower;
        }
        // if we need to increase speed and we are going forward
        else if (requestedPower > currentFlywheelPower) { 
            currentFlywheelPower = currentFlywheelPower + FW_ACCEL_RATE;
            lastFlywheelRampTime = millis();
        }
        // if we need to decrease speed and we are going forward
        else if (requestedPower < currentFlywheelPower) { 
            currentFlywheelPower = currentFlywheelPower - FW_ACCEL_RATE;
            lastFlywheelRampTime = millis();
        }
    }

    return currentFlywheelPower;
}

// Aiming related functions
void Quarterback::aim(QBAim dir) {
  // Debounce for button press
  if (millis() - lastDBElev >= DEBOUNCE_WAIT) {
    // Check which direction the user wants and turn the other direction off
    switch(dir) {
      case AIM_UP: aimingUp = true; aimingDown = false; break;
      case AIM_DOWN: aimingDown = true; aimingUp = false; break;
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
    flywheelMotor.write(rampFW(FLYWHEEL_SPEED_FULL + flywheelSpeedFactor));
  } else {
    flywheelMotor.write(rampFW(FLYWHEEL_STOP_SPEED));
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

void Quarterback::changeFWSpeed(SpeedStatus speed) {
  // Debounce for button press
  if (millis() - lastDBFWChange >= DEBOUNCE_WAIT) {
    // Change the speed factor based on whether the user wants to increase or decrease (HARD CODED)
    switch(speed) {
      case INCREASE: arrayPos++; break;
      case DECREASE: arrayPos--; break;
    }

    // Cap the arrayPos so it doesn't go out of bounds
    arrayPos = constrain(arrayPos, 0, 3);

    // Set the flywheelSpeedFactor
    flywheelSpeedFactor = speedFac[arrayPos];


    // Update the motors if they are spinning for the new speed
    // if (flywheelsOn){
    //   flywheelMotor.write(FLYWHEEL_SPEED_FULL + flywheelSpeedFactor);
    // } else {
    //   flywheelMotor.write(FLYWHEEL_STOP_SPEED);
    // }

    lastDBFWChange = millis();
  }
}