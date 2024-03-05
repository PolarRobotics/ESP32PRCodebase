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

// Gyro Includes
#include <Adafruit_MPU6050.h>
// #include <Adafruit_Sensor.h>
#include <Wire.h>

// Gyroscope
Adafruit_MPU6050 mpu;

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

// Alter
int speedL = 0;
int a_channelL = 35;
int b_channelL = 34;
int encoderLCount = 0;
int rollerover = 10000;
int b_channel_stateL = 0;

void encoderL() {
  
  b_channel_stateL = digitalRead(b_channelL);

  if (b_channel_stateL == 1) {
    if (encoderLCount >= rollerover) {
      encoderLCount = 0;
    } else {
      encoderLCount = encoderLCount + 1;
    }
      
  } else {
    if (encoderLCount == 0) {
      encoderLCount = rollerover;
    } else {
      encoderLCount = encoderLCount - 1;
    }
      
  }
}

// Added
int speedR = 0;
int a_channelR = 36;    // Pin Label VP
int b_channelR = 39;    // Pin Label VN
int encoderRCount = 0;
int b_channel_stateR = 0;

void encoderR() {
  
  b_channel_stateR = digitalRead(b_channelR);

  if (b_channel_stateR == 1) {
    if (encoderRCount >= rollerover) {
      encoderRCount = 0;
    } else {
      encoderRCount = encoderRCount + 1;
    }
      
  } else {
    if (encoderRCount == 0) {
      encoderRCount = rollerover;
    } else {
      encoderRCount = encoderRCount - 1;
    }
      
  }
}

int prev_current_countL = 0;
int rolleroverthreshold = 2000; //this is bases on the fastes speed we expect, if the differace is going to be grater a rollover has likely accured
unsigned long current_timeL = 0;
unsigned long prev_current_timeL = 0; 
float omegaL = 0;

float commanded_power = 0;

int calcSpeedL(int current_count) {
  
  current_timeL = millis();
  
  //first check if the curret count has rolled over
  if (abs(current_count - prev_current_countL) >= rolleroverthreshold) {
    if ((current_count-rolleroverthreshold)>0) {
      omegaL = float ((current_count-rollerover)-prev_current_countL)/(current_timeL-prev_current_timeL);
    } else {
      omegaL = float ((current_count+rollerover)-prev_current_countL)/(current_timeL-prev_current_timeL);
    }
  } else {
    omegaL = float (current_count-prev_current_countL)/(current_timeL-prev_current_timeL);
  }

  prev_current_countL = current_count;
  prev_current_timeL = current_timeL;

  return omegaL*60; // 156.25 for 384, 312.5 for 192, 1250 for 48, 117.1875 for 512, 75 for 800, 60 for 1000, 29.296875 for 2048

}

int prev_current_countR = 0;
unsigned long current_timeR = 0;
unsigned long prev_current_timeR = 0; 
float omegaR = 0;

int calcSpeedR(int current_count) {
  
  current_timeR = millis();
  
  //first check if the curret count has rolled over
  if (abs(current_count - prev_current_countR) >= rolleroverthreshold) {
    if ((current_count-rolleroverthreshold)>0) {
      omegaR = float ((current_count-rollerover)-prev_current_countR)/(current_timeR-prev_current_timeR);
    } else {
      omegaR = float ((current_count+rollerover)-prev_current_countR)/(current_timeR-prev_current_timeR);
    }
  } else {
    omegaR = float (current_count-prev_current_countR)/(current_timeR-prev_current_timeR);
  }

  prev_current_countR = current_count;
  prev_current_timeR = current_timeR;

  return -omegaR*60; // 156.25 for 384, 312.5 for 192, 1250 for 48, 117.1875 for 512, 75 for 800, 60 for 1000, 29.296875 for 2048

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

  // Alter
  pinMode(a_channelL, INPUT_PULLUP);
  pinMode(b_channelL, INPUT);

  attachInterrupt(a_channelL, encoderL, RISING);

  // Added
  pinMode(a_channelR, INPUT_PULLUP);
  pinMode(b_channelR, INPUT);

  attachInterrupt(a_channelR, encoderR, RISING);

  delay(800);
  // Gyro paired?
    if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setGyroRange(MPU6050_RANGE_250_DEG);  // 250, 500, 1000, 2000
  Serial.print("Gyro range set to: " + String(mpu.getGyroRange()));
    
  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);  // 260, 184, 94, 44, 21, 10, 5
  Serial.print("Filter bandwidth set to: " + String(mpu.getFilterBandwidth()));

  ps5.attachOnConnect(onConnection);
  ps5.attachOnDisconnect(onDisconnect);

  delay(1000);
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
  //Serial.print(encoderACount);
  // Alter
  speedL = calcSpeedL(encoderLCount);
  // Added
  speedR = calcSpeedR(encoderRCount);

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  Serial.print("Rotation Z,");
  Serial.print(g.gyro.z - 0.03);
  Serial.print(",rad/s,");

  drive->setCurrentAngelSpeed(g.gyro.z - 0.03);

  //drive->update(speed);

  // delay(5);

  if (ps5.isConnected()) { 
    // Serial.print(F("\r\nConnected"));
    // ps5.setLed(255, 0, 0);   // set LED red

    if (robotType == mecanum_center) {
      ((DriveMecanum*) drive)->setStickPwr(ps5.LStickX(), ps5.LStickY(), ps5.RStickX());
    } else {
      drive->setStickPwr(ps5.LStickY(), ps5.RStickX());
    }

    // determine BSN percentage (boost, slow, or normal)
    if (ps5.Touchpad()){
      drive->emergencyStop();
      drive->setBSN(Drive::BRAKE);
    } else if (ps5.R1()) {
      drive->setBSN(Drive::BOOST);
      // ps5.setLed(0, 255, 0);   // set LED red
    } else if (ps5.L1()) {
      drive->setBSN(Drive::SLOW);
    } else if (ps5.R2() && robotType == runningback) {
      drive->setBSNValue(FALCON_NORMAL_TREVOR_PCT);
    } else {
      drive->setBSN(Drive::NORMAL);
    }

    // Manual LED State Toggle (Defense/Offense)
    if (ps5.Options()) {
      lights.togglePosition();
    }

    if (robotType != lineman && lights.returnStatus() == lights.OFFENSE) { // && robotType != runningback
      if (lights.returnStatus() == lights.OFFENSE && digitalRead(TACKLE_PIN) == LOW) {
        lights.setLEDStatus(Lights::TACKLED);
        lights.tackleTime = millis();
        lights.tackled = true;
      } 
      // debounce the tackle sensor input
      else if ((millis() - lights.tackleTime) >= lights.switchTime && lights.tackled == true) {
        lights.setLEDStatus(Lights::OFFENSE);
        lights.tackled = false;
      }
    }

    //* Update the motors based on the inputs from the controller
    //* Can change functionality depending on subclass, like robot.action()
    // Alter
    drive->update2(speedL, speedR);
    // Added
    //drive->update(speedR);

    //drive->printDebugInfo(); // comment this line out to reduce compile time and memory usage

    //! Performs all special robot actions depending on the instantiated Robot subclass
    robot->action();
      
  } else { // no response from PS5 controller within last 300 ms, so stop
      // Emergency stop if the controller disconnects
      drive->emergencyStop();
      lights.setLEDStatus(Lights::UNPAIRED);
  }
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
