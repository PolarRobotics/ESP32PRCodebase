#include <Arduino.h>
#include <ps5Controller.h> // esp ps5 library

// Custom Polar Robotics Libraries:
#include "PolarRobotics.h"
// #include <Robot/Robot.h>
#include <Drive/Drive.h>
// #include <Robot/Lights.h>

// Robot Libraries:
#if BOT_TYPE == 0    // Lineman
#elif BOT_TYPE == 1  // Receiver
#elif BOT_TYPE == 2  // Old Center
    #include <Robot/Center.h>
    Center centerBot;
#elif BOT_TYPE == 3  // Mecanum Center
#elif BOT_TYPE == 4  // Quarterback
    #include <Robot/Quarterback.h>
    Quarterback qbBot(SPECBOT_PIN1, SPECBOT_PIN2, SPECBOT_PIN3);
#elif BOT_TYPE == 5  // Kicker
    #include <Robot/Kicker.h>
    Kicker kickerBot;
#endif

// Robot and Drivebase
// Since pins are GPIO, location doesnt matter, can be changed




// uint8_t motorType;
Drive DriveMotors;
ps5Controller PS5;
void onDisconnect();
void onConnection();

// Lights robotLED;

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

    // Set the motor type
    #if MOTOR_TYPE == 1    // Small Motor
        DriveMotors.setMotorType(MOTORS::small);
    #else                  // Big Motor
        DriveMotors.setMotorType(MOTORS::big);
    #endif
    
    // Set the main driving motors
    DriveMotors.setServos(LEFT_MOT_PIN, RIGHT_MOT_PIN);

    // Set the special bot type
    #if BOT_TYPE == 0    // Lineman
    #elif BOT_TYPE == 1  // Receiver
    #elif BOT_TYPE == 2  // Old Center
        centerBot.setServos(SPECBOT_PIN1, SPECBOT_PIN2);
    #elif BOT_TYPE == 3  // Mecanum Center
    #elif BOT_TYPE == 4  // Quarterback
        qbBot.setup();
    #elif BOT_TYPE == 5  // Kicker
        kickerBot.setup(SPECBOT_PIN1);
    #endif
 
    // Set initial LED color state
    // robotLED.setupLEDS();
    // robotLED.setLEDStatus(Lights::PAIRING);

    //replace with your controllers MAC address
    PS5.begin("BC:C7:46:03:7A:ED");
    // "BC:C7:46:03:7A:ED" i++
    // "BC:C7:46:03:38:70" sqrt(-1)
    // "BC:C7:46:03:38:72"
    // "BC:C7:46:04:09:62" Actually Rhys's controller
    // Callbacks defined in PolarRobotics.h
    PS5.attachOnConnect(onConnection);
    PS5.attachOnDisconnect(onDisconnect);
}

/*
   __  __      _      ___   _   _     _        ___     ___    ____
  |  \/  |    / \    |_ _| | \ | |   | |      / _ \   / _ \  |  _ \
  | |\/| |   / _ \    | |  |  \| |   | |     | | | | | | | | | |_) |
  | |  | |  / ___ \   | |  | |\  |   | |___  | |_| | | |_| | |  __/
  |_|  |_| /_/   \_\ |___| |_| \_|   |_____|  \___/   \___/  |_|

*/
void loop() {
    // The main looping code, controls driving and any actions during a game
    if (PS5.isConnected()) {
        DriveMotors.setStickPwr(PS5.LStickY(), PS5.RStickX());

        // determine BSN percentage (boost, slow, or normal)
        if (PS5.Touchpad()){
            DriveMotors.emergencyStop();
            DriveMotors.setBSN(Drive::brake);
        } else if (PS5.R1()) {
            DriveMotors.setBSN(Drive::boost);
            PS5.setLed(0, 255, 0);   // set LED red
        } else if (PS5.L1()) {
            DriveMotors.setBSN(Drive::slow);
        } else {
            DriveMotors.setBSN(Drive::normal);
        }

        // if(PS5.getButtonPress(UP)){
        //   robotLED.togglePosition();
        // }
        
        // Update the motors based on the inputs from the controller
        if(PS5.L2()) {
            PS5.setLed(255, 255, 0);   // set LED yellow
            DriveMotors.drift();
        } else {
            DriveMotors.update();
            
            // DriveMotors.printDebugInfo(); // comment this line out to reduce compile time and memory usage
        }
        // Serial.printf("Left: x: %d, y: %d, Right: x: %d, y: %d\n", 
        // PS5.LStickX(), PS5.LStickY(), PS5.RStickX(), PS5.RStickY());

        // Special Bot Actions
        #if BOT_TYPE == 0    // Lineman
        #elif BOT_TYPE == 1  // Receiver
        #elif BOT_TYPE == 2  // Old Center
            // Control the arm of the center
            if (PS5.R2()) {
                centerBot.armControl(armStatus::Higher);
            } else if (PS5.L2()) {
                centerBot.armControl(armStatus::Lower);
            } else if (PS5.Triangle()) {
                centerBot.armControl(armStatus::Hold);
            } else {
                centerBot.armControl(armStatus::Stop);
            }

            // Control the Claw of the center
            if (PS5.Up()) {
                centerBot.clawControl(clawStatus::Open);
            } else if (PS5.Down()) {
                centerBot.clawControl(clawStatus::Close);
            } else {
                centerBot.clawControl(clawStatus::clawStop);
            }
        #elif BOT_TYPE == 3  // Mecanum Center
        #elif BOT_TYPE == 4  // Quarterback
            // Update the bools within the class to see if the user wants to go up or down
            if (PS5.Up())
                qbBot.aim(qbAim::aimUp);
            else if (PS5.Down())
                qbBot.aim(qbAim::aimDown);
            
            // Update the aim on quarterback to see if we need to stop or not
            quarterbackbot.updateAim();

            // Toogle the Conveyor and Flywheels
            if (PS5.Square())
                qbBot.toggleConveyor();
            else if (PS5.Circle())
                qbBot.toggleFlywheels();
            
            // Change the flywheel speed
            if(PS5.Triangle())
                qbBot.changeFWSpeed(speedStatus::increase);
            else if (PS5.Cross())
                qbBot.changeFWSpeed(speedStatus::decrease);
        #elif BOT_TYPE == 5  // Kicker
            // Control the motor on the kicker
            if (PS5.Triangle())
                kickerBot.turnfwd();
            else if (PS5.Cross())
                kickerBot.turnrev();
            else
                kickerBot.stop();
        #endif
        
    } else { // no response from PS5 controller within last 300 ms, so stop
        // Emergency stop if the controller disconnects
        DriveMotors.emergencyStop();
    }
    // DriveMotors.printDebugInfo();
    // robotLED.updateLEDS();

}

/**
 * @brief onConnection: Function to be called on controller connect
 */
void onConnection() {
    if(PS5.isConnected()) {
        Serial.println(F("Controller Connected..."));
        // PS5.setLed(0, 255, 0);   // set LED green
    }
}

/**
 * @brief onDisconnect: Function to be called on controller disconnect
 * this may be extremely helpful in the future to fix bots driving off aimlessly
 * and we theoretically could reconnect to the bot if a bot were to loose power and it was restored 
 * without having to touch the bot
 */
void onDisconnect() {
    Serial.println(F("Controller Disconnected."));
    DriveMotors.emergencyStop();
    // DriveMotors.emergencyStop();
}
