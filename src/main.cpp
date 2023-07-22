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

// Config
ConfigManager config;

// Temp Single Encoder Code
int enc_cntr = 0;

// TaskHandle_t encoderTask;

// void encoderLoop(void* pvParameters) {
//   for (;;) {
//     Serial.print(F("Core: "));
//     Serial.println(xPortGetCoreID());
//   }
// }

#define NUM_ENCODER_EDGES 4
// ENCODER_MOVEMENT_THRESHOLD must be larger than a reasonable maximum revolution period of the wheel in milliseconds
#define ENCODER_MOVEMENT_THRESHOLD_MS 1000
#define RESOLUTION_DIVISION_FACTOR 48

volatile unsigned long left_enc_A_edges[NUM_ENCODER_EDGES];
volatile unsigned long left_enc_B_edges[NUM_ENCODER_EDGES];
volatile unsigned long left_enc_A_periods[NUM_ENCODER_EDGES - 1];
volatile unsigned long left_enc_B_periods[NUM_ENCODER_EDGES - 1];
volatile unsigned long left_enc_A_last_updated = 0L;
volatile unsigned long left_enc_B_last_updated = 0L;
volatile int left_enc_A_cntr = 0;
volatile int left_enc_B_cntr = 0;

void update_period_arr(volatile unsigned long* edgeArr, volatile unsigned long* periodArr) {
  for (int i = 0; i < NUM_ENCODER_EDGES - 1; i++) {
    periodArr[i] = edgeArr[i + 1] - edgeArr[i];
  }
}

unsigned long enc_avg_period(volatile unsigned long* periodArr) {
  unsigned long total = 0L;
  for(int i = 0; i < NUM_ENCODER_EDGES - 1; i++) {
    total += periodArr[i];
  }
  return total / (NUM_ENCODER_EDGES - 1);
}

unsigned long enc_avg_period(volatile unsigned long* edgeArr, volatile unsigned long* periodArr) {
  update_period_arr(edgeArr, periodArr);
  return enc_avg_period(periodArr);
}

void update_edge_arr(volatile unsigned long* arr, unsigned long newVal) {
  for(int i = 1; i < NUM_ENCODER_EDGES; i++) {
    arr[i - 1] = arr[i];
  }
  arr[NUM_ENCODER_EDGES - 1] = newVal;
}

void print_edge_arr(volatile unsigned long* arr) {
  Serial.print(F("[ "));
  for(int i = 0; i < NUM_ENCODER_EDGES; i++) {
    Serial.print(arr[i]);
    if (i < NUM_ENCODER_EDGES - 1) {
      Serial.print(F(", "));
    }
  }  
  Serial.print(F(" ]"));
}

void print_period_arr(volatile unsigned long* arr) {
  Serial.print(F("[ "));
  for(int i = 0; i < NUM_ENCODER_EDGES - 1; i++) {
    Serial.print(arr[i]);
    if (i < NUM_ENCODER_EDGES - 2) {
      Serial.print(F(", "));
    }
  }  
  Serial.print(F(" ]"));
}

void left_enc_A_tick() {
  if (left_enc_A_cntr == RESOLUTION_DIVISION_FACTOR) {
    update_edge_arr(left_enc_A_edges, millis());
    update_period_arr(left_enc_A_edges, left_enc_A_periods);
    left_enc_A_last_updated = millis();
    left_enc_A_cntr = 0;
  } else {
    left_enc_A_cntr++;
  }
}

void left_enc_B_tick() {
  if (left_enc_B_cntr == RESOLUTION_DIVISION_FACTOR) {
    update_edge_arr(left_enc_B_edges, millis());
    update_period_arr(left_enc_B_edges, left_enc_B_periods);
    left_enc_B_last_updated = millis();
    left_enc_B_cntr = 0;
  } else {
    left_enc_B_cntr++;
  }
}

int get_left_rpm() {
  if (millis() - left_enc_A_last_updated > ENCODER_MOVEMENT_THRESHOLD_MS) {
    return 0;
  } else {
    return 60000 / enc_avg_period(left_enc_A_periods);
  }
}

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

  // work backwards from highest ordinal enum since lineman should be default case
  switch (robotType) {
    //* Each case should have the following:
    // An initialization of `robot` as a new Robot subclass
    // An initialization of `drive` as a new Drive subclass
    // A call to drive->setServos (or downcast and call to override)
    // An initialization of `lights` if needed depending on the bot type
    case kicker:
      robot = new Kicker(SPECBOT_PIN1);
      drive = new Drive(kicker, motorType);
      drive->setServos(M1_PIN, M2_PIN);
      break;
    case quarterback:
      robot = new Quarterback(SPECBOT_PIN1, SPECBOT_PIN2, SPECBOT_PIN3);
      drive = new Drive(quarterback, motorType);
      drive->setServos(M1_PIN, M2_PIN);
      break;
    case mecanum_center:
      robot = new MecanumCenter(SPECBOT_PIN1, SPECBOT_PIN2);
      //! TODO: compensate for MecanumCenter.setup()
      drive = new DriveMecanum();
      ((DriveMecanum*) drive)->setServos(M1_PIN, M2_PIN, M3_PIN, M4_PIN);
      break;
    case center:
      robot = new Center(SPECBOT_PIN1, SPECBOT_PIN2);
      drive = new Drive(center, motorType);
      drive->setServos(M1_PIN, M2_PIN);
      break;
    case runningback:
      robot = new Lineman();
      drive = new DriveQuick();
      drive->setServos(M1_PIN, M2_PIN);
      break;
    case receiver:
    case lineman:
    default: // Assume lineman
      robot = new Lineman();
      drive = new Drive(lineman, motorType);
      drive->setServos(M1_PIN, M2_PIN);
  }

  lights.setupLEDS();
  lights.setLEDStatus(Lights::PAIRING);

  //! Activate Pairing Process: this code is BLOCKING, not instantaneous
  activatePairing();

  lights.setLEDStatus(Lights::PAIRED);
  

  if (robotType == kicker) {
    ((Kicker*) robot)->enable();
  }

  ps5.attachOnConnect(onConnection);
  ps5.attachOnDisconnect(onDisconnect);

  // temp single encoder test code
  // pins:
  // 34: Ch A
  // 35: Ch B
  // 36/VP: Ch X
  // Vin - 5V
  // GND - GND
  pinMode(34, INPUT);
  pinMode(35, INPUT);
//   pinMode(36, INPUT);

  attachInterrupt(digitalPinToInterrupt(34), left_enc_A_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(35), left_enc_B_tick, RISING);


  // Creates task from function `encoderLoop` with name "EncoderTask"
  // Stack size 10000 memory words
  // Paramter NULL
  // Priority 0 (lowest)
  // Task handle `&encoderTask`
  // On core ~~0~~ 1
//   xTaskCreatePinnedToCore(encoderLoop, "EncoderTask", 10000, NULL, 0, &encoderTask, 1);

  //! To delete task:
  // vTaskDelete(encoderTask);
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

    Serial.print(F("enc_A: "));
    // print_edge_arr(left_enc_A_edges);
    // Serial.print(left_enc_A_cntr);
    // print_period_arr(left_enc_A_periods);
    Serial.print(enc_avg_period(left_enc_A_edges, left_enc_A_periods));
    Serial.print(F(" | enc_B: "));
    // Serial.print(left_enc_B_cntr);
    // print_edge_arr(left_enc_B_edges);
    // print_period_arr(left_enc_B_periods);
    Serial.print(enc_avg_period(left_enc_B_edges, left_enc_B_periods));
    Serial.print(F(" | rpm: "));
    Serial.print(get_left_rpm());
    Serial.println();
//   if (enc_cntr = 20) {
    // Temp single encoder code
    // Serial.print(F("ENC_A: "));
    // Serial.print(digitalRead(34));
    // Serial.print(F(", ENC_B: "));
    // Serial.print(digitalRead(35));
    // Serial.print(F(", ENC_X: "));
    // Serial.println(digitalRead(36));
    // enc_cntr = 0;
//   } else {
//     enc_cntr++;
//   }

    // Serial.print(F("Core: "));
    // Serial.println(xPortGetCoreID());


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
    } else {
      drive->setBSN(Drive::NORMAL);
    }

    // Manual LED State Toggle (Defense/Offense)
    if (ps5.Options()) {
      lights.togglePosition();
    }

    if (robotType != lineman) {
      if (lights.returnStatus() == lights.OFFENSE && digitalRead(TACKLE_PIN) == LOW) {
        lights.setLEDStatus(Lights::TACKLED);
        lights.tackleTime = millis();
        lights.tackled = true;
      } else if ((millis() - lights.tackleTime) >= lights.switchTime && lights.tackled == true) {
        lights.setLEDStatus(Lights::OFFENSE);
        lights.tackled = false;
      }
    }

    //* Update the motors based on the inputs from the controller
    //* Can change functionality depending on subclass, like robot.action()
    drive->update();
    // drive->printDebugInfo(); // comment this line out to reduce compile time and memory usage

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
}

/**
 * @brief onDisconnect: Function to be called on controller disconnect
 * Stops bots from driving off and ramming into a wall or someone's foot if they disconnect
 */
void onDisconnect() {
    Serial.println(F("Controller Disconnected."));
    drive->emergencyStop();
}
