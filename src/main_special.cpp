#include <Arduino.h>

// Custom Polar Robotics Libraries:
#include <PolarRobotics.h>
// #include <Robot/Robot.h>
#include <Drive/Drive.h>
#include "Robot/Center.h"
#include "Robot/Quarterback.h"
#include "Robot/Kicker.h"

// compiler if structure, solves issue with no on board EEPROM on the esp
#if BOT_TYPE == 2 // center
// Center specific variables  
#define armPin 6
#define clawPin 13
Center centerbot;

#elif BOT_TYPE == 3 // mecanum center

#elif BOT_TYPE == 4 // quarterback
// Quarterback specific variables
#define FLYWHEEL_PIN 7
#define ELEVATION_MOTORS_PIN 4
#define CONVEYOR_MOTOR_PIN 6 
Quarterback quarterbackbot;

#elif BOT_TYPE == 5 // kicker
// Kicker specific variables
#define windupPin 7
Kicker kickerbot;
#else // default to linemen 

#endif

// Drive specific variables and objects
#define LEFT_MOT_PIN 32
#define RIGHT_MOT_PIN 33
const uint8_t motorT = MOTOR_TYPE;

Drive DriveMotors;

/*
   ____    _____   _____   _   _   ____
  / ___|  | ____| |_   _| | | | | |  _ \
  \___ \  |  _|     | |   | | | | | |_) |
   ___) | | |___    | |   | |_| | |  __/
  |____/  |_____|   |_|    \___/  |_|

*/

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.print(F("\r\nStarting..."));
  // DriveMotors.attach();
  DriveMotors.setMotorType((MOTORS) MOTOR_TYPE);
  Serial.print(F("\r\nConnected"));

#if BOT_TYPE == 2 // center
  // the bot is a center (old center)
  centerArm.attach(armPin);
  centerClaw.attach(clawPin);
  centerbot.setServos(centerArm, centerClaw);
  PS5.leftTrigger.setTriggerForce(0, 255);
  PS5.rightTrigger.setTriggerForce(0, 255);
#elif BOT_TYPE == 3 // quarterback
  // the bot is a quarterback
  quarterbackbot.attachMotors(FLYWHEEL_PIN, CONVEYOR_MOTOR_PIN, ELEVATION_MOTORS_PIN);
#elif BOT_TYPE == 4
  // the bot is a kicker 
  DriveMotors.setMotorType(MOTORS::big);
  kickerbot.setup(windupPin);
#else
  // default
#endif
  // Reset PWM on startup
  // analogWrite(LEFT_MOT_PIN, 0);
  // analogWrite(RIGHT_MOT_PIN, 0);
}

/*
   __  __      _      ___   _   _     _        ___     ___    ____
  |  \/  |    / \    |_ _| | \ | |   | |      / _ \   / _ \  |  _ \
  | |\/| |   / _ \    | |  |  \| |   | |     | | | | | | | | | |_) |
  | |  | |  / ___ \   | |  | |\  |   | |___  | |_| | | |_| | |  __/
  |_|  |_| /_/   \_\ |___| |_| \_|   |_____|  \___/   \___/  |_|

*/
void loop() {
 
  // clean up the usb registers, allows for new commands to be executed
  Usb.Task();

  // The main looping code, controls driving and any actions during a game
  if ((millis() - PS5.getLastMessageTime()) < 300) { // checks if PS5 is connected, had response within 300 ms
    DriveMotors.setStickPwr(PS5.getAnalogHat(LeftHatY), PS5.getAnalogHat(RightHatX));

    // determine BSN percentage (boost, slow, or normal)
    if (PS5.getButtonPress(R1)) {
      DriveMotors.setBSN(Drive::boost);
    } else if (PS5.getButtonPress(L1)) {
      DriveMotors.setBSN(Drive::slow);
    } else {
      DriveMotors.setBSN(Drive::normal);
    }

    // Update the motors based on the inputs from the controller  
    DriveMotors.update();

    if (botType == lineman || botType == receiver) {
      

    }
    else if (botType == center) {
      if (PS5.getAnalogButton(R2)) {
        centerbot.armControl(armStatus::Higher);
      } else if (PS5.getAnalogButton(L2)) {
        centerbot.armControl(armStatus::Lower);
      } else if (PS5.getButtonPress(TRIANGLE)) {
        centerbot.armControl(armStatus::Hold);
      } else {
        centerbot.armControl(armStatus::Stop);
      }
      
      if (PS5.getButtonPress(UP)) {
        centerbot.clawControl(clawStatus::Open);
      } else if (PS5.getButtonPress(DOWN)) {
        centerbot.clawControl(clawStatus::Close);
      } else {
        centerbot.clawControl(clawStatus::clawStop);
      }
    }
    else if (botType == quarterback) {
      // Update the bools within the class to see if the user wants to go up or down
      if (PS5.getButtonClick(UP))
        quarterbackbot.aim(qbAim::aimUp);
      else if (PS5.getButtonClick(DOWN))
        quarterbackbot.aim(qbAim::aimDown);
      
      // Update the aim on quarterback to see if we need to stop or not
      quarterbackbot.updateAim();

      // Toogle the Conveyor and Flywheels
      if (PS5.getButtonClick(SQUARE))
        quarterbackbot.toggleConveyor();
      else if (PS5.getButtonClick(CIRCLE))
        {quarterbackbot.toggleFlywheels();}
      
      // Change the flywheel speed
      if(PS5.getButtonClick(TRIANGLE))
        quarterbackbot.changeFWSpeed(speedStatus::increase);
      else if (PS5.getButtonClick(CROSS))
        quarterbackbot.changeFWSpeed(speedStatus::decrease);
    }
    else if (botType == kicker) {
      // Control the motor on the kicker
      if (PS5.getButtonPress(TRIANGLE))
        kickerbot.turnfwd();
      else if (PS5.getButtonPress(CROSS))
        kickerbot.turnrev();
      else
        kickerbot.stop();
    }


  } else { // no response from PS5 controller within last 300 ms, so stop
    // Emergency stop if the controller disconnects
    DriveMotors.emergencyStop();
  }
}
