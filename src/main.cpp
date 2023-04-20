/**
 * @brief Main Code File
 * 
 * The place where the magic happens.
 * This code is what's run by default by the ESP32.
 * `setup()` is called once upon startup. Initialization and preparation occur here.
 * `loop()` is called infinitely after `setup()` completes. Most real-time logic is here.
 **/

#include <Arduino.h>
#include <ps5Controller.h> // ESP PS5 library

// Custom Polar Robotics Libraries:
#include <PolarRobotics.h>
#include <Pairing/pairing.h>
#include <Robot/Lights.h>

// Robot Libraries:
#if BOT_TYPE == 0    // Lineman
    #include <Drive/Drive.h>
    Drive DriveMotors;
#elif BOT_TYPE == 1  // Receiver/Runningback
    #include <Drive/Drive.h>
    Drive DriveMotors;
#elif BOT_TYPE == 2  // Old Center
    #include <Drive/Drive.h>
    #include <Robot/Center.h>
    Drive DriveMotors;
    Center centerBot;
#elif BOT_TYPE == 3  // Mecanum Center
    #include <Drive/DriveMecanum.h>
    #include <Robot/MecanumCenter.h>
    MecanumCenter mcBot(SPECBOT_PIN1, SPECBOT_PIN2);

    DriveMecanum DriveMotors;
#elif BOT_TYPE == 4  // Quarterback
    #include <Drive/Drive.h>
    #include <Robot/Quarterback.h>
    Quarterback qbBot(SPECBOT_PIN1, SPECBOT_PIN2, SPECBOT_PIN3);
    Drive DriveMotors;
#elif BOT_TYPE == 5  // Kicker
    #include <Drive/Drive.h>
    #include <Robot/Kicker.h>
    Kicker kickerBot;
    Drive DriveMotors;
#elif BOT_TYPE == 6 // runningback
    #include <Drive/DriveQuick.h>
    DriveQuick DriveMotors;
#endif

#if BOT_TYPE != 4 | BOT_TYPE != 2 | BOT_TYPE != 3
Lights robotLED;
#endif

// Prototypes for Controller Callbacks
// Implementations located at the bottom of this file
void onConnection();
void onDisconnect();

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
    // Serial.print(F("\r\nStarting..."));
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(TACKLE_PIN, INPUT); // Try INPUT_PULLUP

    // Set the motor type
#if MOTOR_TYPE == 0    // Big Motor
    DriveMotors.setMotorType(MOTORS::big);
#elif MOTOR_TYPE == 1  // Small Motor
    DriveMotors.setMotorType(MOTORS::small);
#elif MOTOR_TYPE == 2             
    DriveMotors.setMotorType(MOTORS::mecanummotor);
#elif MOTOR_TYPE == 3
    DriveMotors.setMotorType(MOTORS::falconmotor);
#endif

// Set the special bot type
#if BOT_TYPE == 0 // Lineman
    DriveMotors.setServos(M1_PIN, M2_PIN);
#elif BOT_TYPE == 1   // Receiver
    DriveMotors.setServos(M1_PIN, M2_PIN);
#elif BOT_TYPE == 2  // Old Center
    DriveMotors.setServos(M1_PIN, M2_PIN);
    centerBot.setServos(SPECBOT_PIN1, SPECBOT_PIN2);
#elif BOT_TYPE == 3  // Mecanum Center
    DriveMotors.setServos(M1_PIN, M2_PIN, M3_PIN, M4_PIN);
    mcBot.setup();
#elif BOT_TYPE == 4  // Quarterback
    DriveMotors.setServos(M1_PIN, M2_PIN);
    qbBot.setup();
#elif BOT_TYPE == 5  // Kicker
    DriveMotors.setServos(M1_PIN, M2_PIN);
    kickerBot.setup(SPECBOT_PIN1);
#elif BOT_TYPE == 6  // Runningback
    DriveMotors.setServos(M1_PIN, M2_PIN);
#endif
 
    // Set initial LED color state
    #if BOT_TYPE != 4 | BOT_TYPE != 2 | BOT_TYPE != 3
    robotLED.setupLEDS();
    // robotLED.setLEDStatus(Lights::PAIRING);
    robotLED.setLEDStatus(Lights::PAIRING);
    activatePairing();
    robotLED.setLEDStatus(Lights::PAIRED);
    #endif

    // Serial.print(F("\r\nConnected"));

    ps5.attachOnConnect(onConnection);
    ps5.attachOnDisconnect(onDisconnect);
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
        // Serial.print(F("\r\nConnected"));
        // ps5.setLed(255, 0, 0);   // set LED red

    #if BOT_TYPE == 3 // temporary solution
        DriveMotors.setStickPwr(ps5.LStickX(), ps5.LStickY(), ps5.RStickX());
    #else //if BOT_TYPE != 3
        DriveMotors.setStickPwr(ps5.LStickY(), ps5.RStickX());
    #endif


        // determine BSN percentage (boost, slow, or normal)
        if (ps5.Touchpad()){
            DriveMotors.emergencyStop();
            DriveMotors.setBSN(Drive::brake);
        } else if (ps5.R1()) {
            DriveMotors.setBSN(Drive::boost);
            // ps5.setLed(0, 255, 0);   // set LED red
        } else if (ps5.L1()) {
            DriveMotors.setBSN(Drive::slow);
        } else {
            DriveMotors.setBSN(Drive::normal);
        }

        // Manual LED State Toggle (Defense/Offense)
        #if BOT_TYPE != 4 | BOT_TYPE != 2 | BOT_TYPE != 3
        if(ps5.Options()){
            robotLED.togglePosition();
        }

        #if BOT_TYPE != 0 // if the bot is not a linemen
        // Update the LEDs based on tackle (tPin input) for offensive robot
        if(robotLED.returnStatus() == robotLED.OFFENSE && digitalRead(TACKLE_PIN) == LOW) {
            // robotLED.setLEDStatus(Lights::DEFAULTL);
            robotLED.setLEDStatus(Lights::TACKLED);
            tackleTime = millis();
            tackled = true;
        } 
        // Switch the LED state back to offense after being tackled a certain amount of time
        else if((millis() - tackleTime) >= switchTime && tackled == true){
            robotLED.setLEDStatus(Lights::OFFENSE);
            tackled = false;
        }
        #endif
        #endif
        
        // Update the motors based on the inputs from the controller
        if(ps5.L2()) { 
            // ps5.setLed(255, 255, 0);   // set LED yellow
            DriveMotors.drift();
        } else {
            DriveMotors.update();
            
            DriveMotors.printDebugInfo(); // comment this line out to reduce compile time and memory usage
        }
        // Serial.printf("Left: x: %d, y: %d, Right: x: %d, y: %d\n", 
        // PS5.LStickX(), PS5.LStickY(), PS5.RStickX(), PS5.RStickY());

    // Special Bot Actions
    #if BOT_TYPE == 0    // Lineman
    #elif BOT_TYPE == 1  // Receiver
    #elif BOT_TYPE == 2  // Old Center
        // Control the arm of the center
        if (ps5.Triangle()) {
            centerBot.armControl(armStatus::Higher);
        } else if (ps5.Cross()) {
            centerBot.armControl(armStatus::Lower);
        } else if (ps5.Circle()) {
            centerBot.armControl(armStatus::Hold);
        } else {
            centerBot.armControl(armStatus::Stop);
        }

        // Control the Claw of the center
        if (ps5.Up()) {
            centerBot.clawControl(clawStatus::Open);
        } else if (ps5.Down()) {
            centerBot.clawControl(clawStatus::Close);
        } else {
            centerBot.clawControl(clawStatus::clawStop);
        }
    #elif BOT_TYPE == 3  // Mecanum Center
        // Toggle the Conveyor and Flywheels
        if (ps5.Square())
            mcBot.toggleConveyor();
        else if (ps5.Circle())
            mcBot.toggleFlywheels();
        
        // Change the flywheel speed
        if(ps5.Triangle())
            mcBot.changeFWSpeed(speedStatus::increase);
        else if (ps5.Cross())
            mcBot.changeFWSpeed(speedStatus::decrease);
    #elif BOT_TYPE == 4  // Quarterback
        // Update the bools within the class to see if the user wants to go up or down
        if (ps5.Up())
            qbBot.aim(qbAim::aimUp);
        else if (ps5.Down())
            qbBot.aim(qbAim::aimDown);
        
        // Toogle the Conveyor and Flywheels
        if (ps5.Square())
            qbBot.toggleConveyor();
        else if (ps5.Circle())
            qbBot.toggleFlywheels();
        
        // Change the flywheel speed
        if(ps5.Triangle())
            qbBot.changeFWSpeed(speedStatus::increase);
        else if (ps5.Cross())
            qbBot.changeFWSpeed(speedStatus::decrease);
        

        // Update the aim and flywheels on quarterback to see if we need to stop or not
        qbBot.update();
    #elif BOT_TYPE == 5  // Kicker
        // Control the motor on the kicker
        if (ps5.Triangle())
            kickerBot.turnfwd();
        else if (ps5.Cross())
            kickerBot.turnrev();
        else
            kickerBot.stop();
    #endif
        
    } else { // no response from PS5 controller within last 300 ms, so stop
        // Emergency stop if the controller disconnects
        DriveMotors.emergencyStop();
        robotLED.setLEDStatus(Lights::NOTPAIRED);
    }
    // robotLED.updateLEDS();
}

/**
 * @brief onConnection: Function to be called on controller connect
 */
void onConnection() {
    if(ps5.isConnected()) {
        Serial.println(F("Controller Connected."));
        // ps5.setLed(0, 255, 0);   // set LED green
        robotLED.setLEDStatus(Lights::PAIRED);
    }
}

/**
 * @brief onDisconnect: Function to be called on controller disconnect
 * Stops bots from driving off and ramming into a wall or someone's foot if they disconnect
 */
void onDisconnect() {
    Serial.println(F("Controller Disconnected."));
    DriveMotors.emergencyStop();
}

/// @brief Allows accessing LED state from other files like `pairing.cpp`
/// Used to avoid circular dependencies
extern void extUpdateLEDs() {
  robotLED.updateLEDS();
}