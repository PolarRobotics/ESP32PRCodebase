#include "MecanumCenter.h"

MecanumCenter::MecanumCenter(uint8_t fwpin, uint8_t conveyorpin) {
  // Declare that the flywheels are off
  flywheelsOn = false;

  // Declare the the conveyor is off
  conveyorOn = false;

  // Label the pins inside the class
  this->flywheelPin = fwpin;
  this->conveyorPin = conveyorpin;

  // Attach the motors inside the class to their respective pins
  flywheelMotor.setup(flywheelPin);
  conveyorMotor.setup(conveyorPin);

  // Set the motor to zero so it doesnt spin on startup
  conveyorMotor.write(MC_CONVEYOR_OFF);
  
  // spin on startup for testing
  // flywheelMotor.write(MC_FLYWHEEL_SPEED_FULL);
  // conveyorMotor.write(MC_CONVEYOR_ON);
}

void MecanumCenter::action() {
  // Toggle the Conveyor and Flywheels
  if (ps5.Square())
    toggleConveyor();
  
  if (ps5.Circle())
    toggleFlywheels();
  
  // Change the flywheel speed
  if(ps5.Triangle())
    changeFWSpeed(SpeedStatus::INCREASE);
  else if (ps5.Cross())
    changeFWSpeed(SpeedStatus::DECREASE);

  // debug();
}


void MecanumCenter::toggleFlywheels() {
  if (millis() - lastDBFW >= MC_DEBOUNCE_WAIT) {
    // Toggle the flywheels and use the speed factor to know what speed
    if (!flywheelsOn){
      flywheelMotor.write(MC_FLYWHEEL_SPEED_FULL + flywheelSpeedFactor);
      // Serial.println(F("Write FW ON"));
    } else {
      flywheelMotor.write(MC_FLYWHEEL_STOP_SPEED);
      // Serial.println(F("Write FW OFF"));
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
      // Serial.println(F("Write CONV ON"));
    } else {
      conveyorMotor.write(MC_CONVEYOR_OFF);
      // Serial.println(F("Write CONV OFF"));
    }
    // Toggle the bool so we know which mode it is in
    conveyorOn = !conveyorOn;
    lastDBConv = millis();
  }
}

void MecanumCenter::changeFWSpeed(SpeedStatus speed) {
  // Debounce for button press
  if (millis() - lastDBFWChange >= MC_DEBOUNCE_WAIT) {
    // Change the speed factor based on whether the user wants to increase or decrease
    switch(speed) {
      case INCREASE: flywheelSpeedFactor += 0.05; break;
      case DECREASE: flywheelSpeedFactor -= 0.05; break;
    }
    // Cap it so they only have two levels to speed up and two levels to slow down
    flywheelSpeedFactor = constrain(flywheelSpeedFactor, -0.15, 0.15);

    // Update the motors if they are spinning for the new speed
    if (flywheelsOn) {
      flywheelMotor.write(MC_FLYWHEEL_SPEED_FULL + flywheelSpeedFactor);
      // Serial.println(F("Write FW SPEED CHANGE"));
    } else {
      flywheelMotor.write(MC_FLYWHEEL_STOP_SPEED);
      // Serial.println(F("Write FW STOP"));
    }

    lastDBFWChange = millis();
  }
}

void MecanumCenter::debug() {
  Serial.print(F("fw on: "));
  Serial.print(flywheelsOn);
  Serial.print(F(", conv on: "));
  Serial.println(conveyorOn);
}