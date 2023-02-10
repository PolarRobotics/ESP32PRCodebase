#include <Arduino.h>
// #include <ESP32Servo.h>
// #include <Servo.h>
#include <ps5Controller.h> // new esp ps5 library

// Custom Polar Robotics Libraries:
// #include "PolarRobotics.h"
// #include <Robot/Robot.h>
#include <Drive/Drive.h>
// #include <Robot/Lights.h>

// Robot and Drivebase 
#define lPin 14 //GPIO0
#define rPin 16 //GPIO2
uint8_t motorType;
Drive DriveMotors;
ps5Controller PS5;

// Lights robotLED;

/**
 * @brief onConnection: Function to be called on controller connect
 */
void onConnection() {
    if(PS5.isConnected()) {
        Serial.println(F("Controller Connected."));
        PS5.setLed(0, 255, 0);   // set LED green
    }
}

/**
 * @brief onDisconnect: Function to be called on controller disconnect
 * this may be extremely helpful in the future to fix bots driving off aimlessly in the future
 * and we theoretically could reconnect to the bot if a bot were to loose power and it was restored 
 * without having to touch the bot
 */
void onDisconnect() {
    Serial.println(F("Controller Disconnected."));
    DriveMotors.emergencyStop();
    // DriveMotors.emergencyStop();
}

/*
   ____    _____   _____   _   _   ____
  / ___|  | ____| |_   _| | | | | |  _ \
  \___ \  |  _|     | |   | | | | | |_) |
   ___) | | |___    | |   | |_| | |  __/
  |____/  |_____|   |_|    \___/  |_|

*/

// ESP32PWM pwm;

void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200);
    Serial.print(F("\r\nStarting..."));

    DriveMotors.setMotorType(MOTORS::big);
    DriveMotors.setServos(lPin, rPin);

    
    // Set initial LED color state
    // robotLED.setupLEDS();
    // robotLED.setLEDStatus(Lights::PAIRING);

    
    /*
    //replace with your MAC address "bc:c7:46:04:09:62"
    need better good method of generating a mac address
    the ps5 controller stores the mac address of the device it is paired with
    since there is not pairing protocol yet, you need to use the mac address 
    of a device it is already paired with
    */ 
    PS5.begin("14:2d:4d:2f:11:b4"); 

    Serial.print(F("\r\nConnected"));

    PS5.attachOnConnect(onConnection);
    PS5.attachOnDisconnect(onDisconnect);
    // Reset PWM on startup
    // analogWrite(lPin, 0);
    // analogWrite(rPin, 0);

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
            
            DriveMotors.printDebugInfo(); // comment this line out to reduce compile time and memory usage
        }
        // Serial.printf("Left: x: %d, y: %d, Right: x: %d, y: %d\n", 
        //     PS5.LStickX(), PS5.LStickY(), PS5.RStickX(), PS5.RStickY());
        
    } else { // no response from PS5 controller within last 300 ms, so stop
        // Emergency stop if the controller disconnects
        DriveMotors.emergencyStop();
    }
//   DriveMotors.printDebugInfo();
    // robotLED.updateLEDS();

}
