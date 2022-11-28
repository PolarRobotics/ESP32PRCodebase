#include <Robot/Robot.h>
#include <Drive/Drive.h>
#include <ESP32Servo.h>

// Flywheel defines 
#define FLYWHEEL_SPEED_FULL 120 // this should be between 90 and 140. 
#define FLYWHEEL_STOP_SPEED 93

// Elevation (linear actuators) defines
#define SERVO_SPEED_UP 175
#define SERVO_SPEED_STOP 90 // this should always be 90.
#define SERVO_SPEED_DOWN 5
#define MAX_ELEVATION 100
#define ELEVATION_PERIOD 3750

// Conveyor defines
#define CONVEYOR_ON 140
#define CONVEYOR_OFF 93 

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
        Servo FWMotor;
        Servo conveyorMotor;
        Servo elevationMotors;
        bool flywheelsOn, conveyorOn;
        bool aimingUp, aimingDown;
        bool raise, lower;
        uint8_t currentElevation, targetElevation;
        unsigned long lastElevationTime;
        unsigned long lastFlywheelToggleTime;
        float flywheelSpeedFactor;
    public:
        Quarterback();
        void attachMotors(uint8_t fwpin, 
            uint8_t conveyorpin, uint8_t elevationpin);
        void toggleFlywheels();
        void aim(qbAim dir);
        void toggleConveyor();
        void changeFWSpeed(speedStatus speed);
        void updateAim();
};

Quarterback::Quarterback() {
    // Declare that the flywheels are off
    flywheelsOn = false;

    // Declare the the conveyor is off
    conveyorOn = false;

    // Declare that we are not trying to aim up or down and we are at the current elevation of 0
    aimingUp = false;
    aimingDown = false;
    currentElevation = 0;
}

void Quarterback::attachMotors(uint8_t fwpin, uint8_t conveyorpin, uint8_t elevationpin) {
    // Label the pins inside the class
    m_FlywheelPin = fwpin;
    m_conveyorPin = conveyorpin;
    m_ElevationPin = elevationpin;

    // Attach the motors inside the class to their respective pins
    FWMotor.attach(m_FlywheelPin);
    conveyorMotor.attach(m_conveyorPin);
    elevationMotors.attach(m_ElevationPin);

    // Set the motor to zero so it doesnt spin on startup
    conveyorMotor.write(CONVEYOR_OFF);

    // Lower the Linear Actuators at start up so they are in the bottom position
    elevationMotors.write(SERVO_SPEED_DOWN);
    delay(8000);
    elevationMotors.write(SERVO_SPEED_STOP);
}


void Quarterback::toggleFlywheels() {
    // Toggle the flywheels and use the speed factor to know what speed
    if (!flywheelsOn){
      FWMotor.write(FLYWHEEL_SPEED_FULL + flywheelSpeedFactor);
    } else {
      FWMotor.write(FLYWHEEL_STOP_SPEED);
    }
    // Toggle the bool so we know if its on or not
    flywheelsOn = !flywheelsOn;
}

// Aiming related functions

void Quarterback::aim(qbAim dir) {
  // Check which direction the user wants and turn the other direction off
  switch(dir) {
    case aimUp: aimingUp = true; aimingDown = false; break;
    case aimDown: aimingDown = true; aimingUp = false; break;
  }
}

void Quarterback::updateAim() { 
    // If it is currently moving then dont go into the loop until it is done moving
    if (millis() - lastElevationTime >= ELEVATION_PERIOD) {
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
    // Toggle the conveyor between on or off
    if (!conveyorOn){
      conveyorMotor.write(CONVEYOR_ON);
    } else {
      conveyorMotor.write(CONVEYOR_OFF);
    }
    // Toggle the bool so we know which mode it is in
    conveyorOn = !conveyorOn;
}

void Quarterback::changeFWSpeed(speedStatus speed) {
  // Change the speed factor based on whether the user wants to increase or decrease
  switch(speed) {
    case increase: flywheelSpeedFactor += 5; break;
    case decrease: flywheelSpeedFactor -= 5; break;
  }
  // Cap it so they only have two levels to speed up and two levels to slow down
  flywheelSpeedFactor = constrain(flywheelSpeedFactor, -15, 15);

  // Update the motors if they are spinning for the new speed
  if (flywheelsOn){
      FWMotor.write(FLYWHEEL_SPEED_FULL + flywheelSpeedFactor);
    } else {
      FWMotor.write(FLYWHEEL_STOP_SPEED);
    }

}

// void Quarterback::passBall() {
//     //turn flywheels on to low: approx 10 power for a light boost
//     rightFlywheelMotor.write(FLYWHEEL_STOP_SPEED - 10);
//     leftFlywheelMotor.write(FLYWHEEL_STOP_SPEED + 10);
//     //once firing mechanism is finished add that in and make it a macro?
// }


// void Quarterback::update() {

// }

// void Quarterback::fireWeapon(String requestedStatus) {
//   if (requestedStatus == "Fire") {
//     FireMotor.write(50);
//   } else if (requestedStatus == "Retract") {
//     FireMotor.write(130);
//   } else if (requestedStatus == "Stop") {
//     FireMotor.write(90);
//   }
// }


/*
    Notes:

    IDEA:
    can have an action where if you hold down on a button
    and move the left hat you can adjust the elevation of the QB
    this can allow for better control of the height

*/

/*  OLD MAIN CODE:

    if (PS3.getButtonClick(TRIANGLE) && targetElevation + 1 < 3) {
      targetElevation = targetElevation + 1;
    } else if (PS3.getButtonClick(CROSS) && targetElevation - 1 >= 0) {
      targetElevation = targetElevation - 1;
    } else if (PS3.getButtonClick(DOWN)) {
      targetElevation = 0;
    }

    int maxCounter = 13000;

    if ((counter > maxCounter || counter < 0) && (aimingup == true || aimingdown == true)) {
      aimingup = false;
      aimingdown = false;
      ElevationMotor.write(getSpeedStop());
      if (counter == -1) {
        counter = 0;
      } else {
        counter = maxCounter;
      }
      Serial.println("im stuck");
    } else if (targetElevation > currentElevation) {
      ElevationMotor.write(getSpeedUp());
      currentElevation = targetElevation;
      aimingup = true;
      aimingdown = false;
    } else if (targetElevation < currentElevation) {
      ElevationMotor.write(getSpeedDown());
      currentElevation = targetElevation;
      aimingdown = true;
      aimingup = false;
    } else if (aimingup == true) {
      counter = counter + 1;
    } else if (aimingdown == true) {
      counter = counter - 1;
    }

    if (targetElevation == 1 && counter == maxCounter/2) {
      aimingup = false;
      aimingdown = false;
      ElevationMotor.write(getSpeedStop());
    }

    if (PS3.getButtonClick(LEFT)) {
      ElevationMotor.write(getSpeedDown());
      delay(8000);
      ElevationMotor.write(getSpeedStop());
      counter = 0;
      currentElevation = 0;
      targetElevation = 0;
    }

    if (PS3.getButtonPress(CIRCLE)) {
      conveyor.write(145);
    } else {
      conveyor.write(30);
    }

    // flywheels.write(60);
    
    if (PS3.getButtonClick(SQUARE)) {
      flywheelstate = flywheelstate + 1;
      if (flywheelstate == 1) {
        flywheels.write(100);
        //flywheelstatis = true;
        //Serial.print("ran line 1");
        //Serial.println("  ");
      } else if (flywheelstate == 2) {
        flywheels.write(145);
        //flywheelstatis = true;
        //Serial.print("ran line 1");
        //Serial.println("  ");
      } else if (flywheelstate==3){
        flywheels.write(93);
        //flywheelstatis = false;
        flywheelstate = 0;
        //Serial.print("ran line 2");
        //Serial.println("  ");
      }
        //Serial.print("ran line 3");
        //Serial.println("  ");
    }


*/
