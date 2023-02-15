#include <Arduino.h>
// #include <ESP32Servo.h>
#include <Servo.h>
#include <ps5Controller.h> // new esp ps5 library

// Custom Polar Robotics Libraries:
// #include "PolarRobotics.h"
// #include <Robot/Robot.h>
#include <Drive/Drive.h>
// #include <Robot/Lights.h>

// Robot and Drivebase 
#define lPin 32 //GPIO0
#define rPin 33 //GPIO2
Servo leftMotor;
Servo rightMotor;
uint8_t motorType;
Drive DriveMotors;

// Lights robotLED;

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
    // ESP32PWM::allocateTimer(0);
    // ESP32PWM::allocateTimer(1);
    Serial.begin(115200);
    Serial.print(F("\r\nStarting..."));

    DriveMotors.setMotorType(MOTORS::big);

    leftMotor.attach(lPin, 1, 0, 180, 1000, 2000);
    rightMotor.attach(rPin, 2, 0, 180, 1000, 2000);
    // pwm.attachPin(27, 10000);
    DriveMotors.setServos(leftMotor, rightMotor);

    // leftMotor.setPeriodHertz(50);
    // rightMotor.setPeriodHertz(50);
    
    
    // Set initial LED color state
    // robotLED.setupLEDS();
    // robotLED.setLEDStatus(Lights::PAIRING);

    //replace with your MAC address "bc:c7:46:04:09:62"

    /*
    need better good method of generating a mac address
    the ps5 controller stores the mac address of the device it is paired with
    since there is not pairing protocol yet, you need to use the mac address 
    of a device it is already paired with
    */ 
    ps5.begin("14:2d:4d:2f:11:b4"); 

    Serial.print(F("\r\nConnected"));

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
    if (ps5.isConnected()) {
        ps5.setLed(255, 0, 0);   // set LED red

        DriveMotors.setStickPwr(ps5.LStickY(), ps5.RStickX());

        // determine BSN percentage (boost, slow, or normal)
        if (ps5.Touchpad()){
        DriveMotors.emergencyStop();
        DriveMotors.setBSN(Drive::brake);
        } else if (ps5.R1()) {
        DriveMotors.setBSN(Drive::boost);
        } else if (ps5.L1()) {
        DriveMotors.setBSN(Drive::slow);
        } else {
        DriveMotors.setBSN(Drive::normal);

        }

        // if(PS5.getButtonPress(UP)){
        //   robotLED.togglePosition();
        // }
        
        // Update the motors based on the inputs from the controller
        if(ps5.L2()) {
        DriveMotors.drift();
        } else {
        DriveMotors.update();
        DriveMotors.printDebugInfo();
        }
        // Serial.printf("Left: x: %d, y: %d, Right: x: %d, y: %d\n", 
        //     ps5.LStickX(), ps5.LStickY(), ps5.RStickX(), ps5.RStickY());
        
    } else { // no response from PS5 controller within last 300 ms, so stop
        // Emergency stop if the controller disconnects
        ps5.setLed(255, 255, 0);   // set LED yellow
        DriveMotors.emergencyStop();
    }
//   DriveMotors.printDebugInfo();
    // robotLED.updateLEDS();

}
