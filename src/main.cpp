/**
 * @brief Main Code File
 * 
 * The place where the magic happens.
 * This code is what's run by default by the ESP32.
 * `setup()` is called once upon startup. Initialization and preparation occur here.
 * `loop()` is called infinitely after `setup()` completes. Most real-time logic is here.
 **/

#include <Arduino.h>
#include <ps5Controller.h> // ESP PS5 library, access using global instance `ps5`

// Custom Polar Robotics Libraries:
#include <PolarRobotics.h>
#include <Pairing/pairing.h>
#include <Utilities/ConfigManager.h>

// Robot Includes
#include <Robot/Lights.h>
#include <Robot/Robot.h>
#include <Robot/Lineman.h>
#include <Robot/Center.h>
#include <Robot/MecanumCenter.h>
#include <Robot/Kicker.h>
#include <Robot/Quarterback.h>

// Drive Includes
#include <Drive/Drive.h>
#include <Drive/DriveMecanum.h>
#include <Drive/DriveQuick.h>

// Primary Parent Component Pointers
Robot* robot = nullptr; // subclassed if needed
Drive* drive = nullptr; // subclassed if needed
Lights& lights = Lights::getInstance();

//* How to use subclasses: ((SubclassName*) robot)->function()
//! You must downcast each time you use a special function

// Robot Information from EEPROM/Preferences
BotType robotType;
MotorType motorType;
drive_param_t driveParams;

// Config
ConfigManager config;

// Prototypes for Controller Callbacks
// Implementations located at the bottom of this file
void onConnection();
void onDisconnect();

int speed = 0;
int a_channel = 35;
int b_channel = 34;
int encoderACount = 0;
int rollerover = 2048;
int b_channel_state = 0;

void encoderA() {
  Serial.print("here");
  b_channel_state = digitalRead(b_channel);

  if (b_channel_state == 1) {
    if (encoderACount >= rollerover) {
      encoderACount = 0;
    } else {
      encoderACount = encoderACount + 1;
    }
      
  } else {
    if (encoderACount == 0) {
      encoderACount = rollerover;
    } else {
      encoderACount = encoderACount - 1;
    }
      
  }
}

int prev_current_count = 0;
int rolleroverthreshold = 500; //this is bases on the fastes speed we expect, if the differace is going to be grater a rollover has likely accured
unsigned long current_time = 0;
unsigned long prev_current_time = 0; 
float omega = 0;

float commanded_power = 0;

int calcSpeed(int current_count) {
  
  current_time = millis();
  
  //first check if the curret count has rolled over
  if (abs(current_count - prev_current_count) >= rolleroverthreshold) {
    if ((current_count-rolleroverthreshold)>0) {
      omega = float ((current_count-rollerover)-prev_current_count)/(current_time-prev_current_time);
    } else {
      omega = float ((current_count+rollerover)-prev_current_count)/(current_time-prev_current_time);
    }
  } else {
    omega = float (current_count-prev_current_count)/(current_time-prev_current_time);
  }

  prev_current_count = current_count;
  prev_current_time = current_time;

  return omega*312.5f; // 156.25 for 384, 312.5 for 192, 1250 for 48

}


/*
   ____    _____   _____   _   _   ____
  / ___|  | ____| |_   _| | | | | |  _ \
  \___ \  |  _|     | |   | | | | | |_) |
   ___) | | |___    | |   | |_| | |  __/
  |____/  |_____|   |_|    \___/  |_|

*/

// runs once at the start of the program
void setup() {
  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(TACKLE_PIN, INPUT); // Try INPUT_PULLUP

  // Read robot info from "EEPROM" (ESP32 Preferences) using ConfigManager
  config.read();
  Serial.println(config.toString());
  robotType = config.getBotType();
  motorType = config.getMotorType();
  driveParams = config.getDriveParams();
  // gearRatio = config.getGearRatio();
  // wheelBase = config.getWheelBase();


  // work backwards from highest ordinal enum since lineman should be default case
  switch (robotType) {
    //* Each case should have the following:
    // An initialization of `robot` as a new Robot subclass
    // An initialization of `drive` as a new Drive subclass
    // A call to drive->setupMotors (or downcast and call to override)
    // An initialization of `lights` if needed depending on the bot type
    case kicker:
      robot = new Kicker(SPECBOT_PIN1);
      drive = new Drive(kicker, motorType, driveParams);
      drive->setupMotors(M1_PIN, M2_PIN);
      break;
    case quarterback_old:
      robot = new Quarterback(SPECBOT_PIN1, SPECBOT_PIN2, SPECBOT_PIN3);
      drive = new Drive(quarterback_old, motorType, driveParams);
      drive->setupMotors(M1_PIN, M2_PIN);
      break;
    case mecanum_center:
      robot = new MecanumCenter(SPECBOT_PIN1, SPECBOT_PIN2);
      //! TODO: compensate for MecanumCenter.setup()
      drive = new DriveMecanum();
      ((DriveMecanum*) drive)->setupMotors(M1_PIN, M2_PIN, M3_PIN, M4_PIN);
      break;
    case center:
      robot = new Center(SPECBOT_PIN1, SPECBOT_PIN2);
      drive = new Drive(center, motorType, driveParams);
      drive->setupMotors(M1_PIN, M2_PIN);
      break;
    case runningback:
      robot = new Lineman();
      drive = new DriveQuick(driveParams);
      drive->setupMotors(M1_PIN, M2_PIN);
      break;
    case receiver:
    case lineman:
    default: // Assume lineman
      robot = new Lineman();
      drive = new Drive(lineman, motorType, driveParams);
      drive->setupMotors(M1_PIN, M2_PIN);
  }

  // drive->printSetup();

  // Set up and initialize lights for pairing process
  lights.setupLEDS();
  lights.setLEDStatus(Lights::PAIRING);

  //! Activate Pairing Process: this code is BLOCKING, not instantaneous
  activatePairing();

  // Once paired, set lights to appropriate status
  lights.setLEDStatus(Lights::PAIRED);
  
  // Kicker safety enable once paired
  if (robotType == kicker) {
    ((Kicker*) robot)->enable();
  }

  pinMode(a_channel, INPUT_PULLUP);
  pinMode(b_channel, INPUT);

  attachInterrupt(a_channel, encoderA, RISING);

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

// runs continuously after setup(). controls driving and any special robot functionality during a game
void loop() {

  speed = calcSpeed(encoderACount);
  Serial.print(encoderACount);
  drive->update(speed);

  delay(50);

  // if (ps5.isConnected()) {
  //   // Serial.print(F("\r\nConnected"));
  //   // ps5.setLed(255, 0, 0);   // set LED red

    // if (ps5.Share()) 
    //   lights.setLEDStatus(Lights::DISCO);
    
    // // Manual LED State Toggle (Home/Away/Off)
    // if (ps5.Options()) 
    //   lights.togglePosition();
    
    // // If the robot is able to hold the ball, it is able to be tackled:
    // if (robotType == receiver || robotType == quarterback_old || robotType == runningback) {
    //   // if the lights are in the home or away state and the tackle pin goes low (tackle sensor is active low), enter the tackled state
    //   if ((lights.returnStatus() == Lights::HOME || lights.returnStatus() == Lights::AWAY) && digitalRead(TACKLE_PIN) == LOW) {
    //     lights.setLEDStatus(Lights::TACKLED);
    //     lights.tackleTime = millis();
    //   } 
    //   // leave the tackled state after some time and the tackle sensor pin went back to high
    //   else if ((millis() - lights.tackleTime) >= lights.switchTime && 
    //       lights.returnStatus() == Lights::TACKLED && digitalRead(TACKLE_PIN) == HIGH) {
    //     switch (lights.homeStatus()) {
    //       case Lights::HOME: lights.setLEDStatus(Lights::HOME); break;
    //       case Lights::AWAY: lights.setLEDStatus(Lights::AWAY); break;
    //       case Lights::OFF:  lights.setLEDStatus(Lights::OFF);  break;
    //     }
    //   }
    // }

    // //* Update the motors based on the inputs from the controller
    // //* Can change functionality depending on subclass, like robot.action()
    // drive->update();

    // if (lights.returnStatus() == lights.DISCO && ((millis() - lights.updateTime) >= lights.updateSwitchTime)) {
    //   lights.updateLEDS();
    //   lights.updateTime = millis();
    // }
    // //! Performs all special robot actions depending on the instantiated Robot subclass
    // robot->action();

    // // DEBUGGING:  
    // // drive->printDebugInfo(); // comment this line out to reduce compile time and memory usage
    // // drive->printCsvInfo(); // prints info to serial monitor in a csv (comma separated value) format
    // // lights.printDebugInfo();

    // delay(5); // necessary for lights to be happy
      
  // } else { // no response from PS5 controller within last 300 ms, so stop
  //     // Emergency stop if the controller disconnects
  //     drive->emergencyStop();
  //     lights.setLEDStatus(Lights::UNPAIRED);
  // }
}

/**
 * @brief onConnection: Function to be called on controller connect
 */
void onConnection() {
  if(ps5.isConnected()) {
    Serial.println(F("Controller Connected."));
    // ps5.setLed(0, 255, 0);   // set LED green
    lights.setLEDStatus(Lights::PAIRED);
  }
  drive->emergencyStop();
}

/**
 * @brief onDisconnect: Function to be called on controller disconnect
 * Stops bots from driving off and ramming into a wall or someone's foot if they disconnect
 */
void onDisconnect() {
    Serial.println(F("Controller Disconnected."));
    drive->emergencyStop();
}
