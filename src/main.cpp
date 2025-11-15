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
#include <Robot/QuarterbackBase.h>
#include <Robot/QuarterbackTurret.h>

// Drive Includes
#include <Drive/Drive.h>
#include <Drive/DriveMecanum.h>

// RTOS Includes
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

// Primary Parent Component Pointers
Robot* robot = nullptr; // subclassed if needed
Drive* drive = nullptr; // subclassed if needed
Lights& lights = Lights::getInstance();

//* How to use subclasses: ((SubclassName*) robot)->function()
//! You must downcast each time you use a special function

// Robot Information from EEPROM/Preferences
BotType robotType;
drive_param_t driveParams;

// Config
ConfigManager config;

// Encoder variables
#define ENCODER_EN true

#if ENCODER_EN

#define ENC1_CHA 3
#define ENC1_CHB 4
#define ENC2_CHA 5
#define ENC2_CHB 6
TaskHandle_t isrTaskHandle = NULL;
QueueHandle_t queueA = xQueueCreate(10, sizeof(uint32_t));
QueueHandle_t queueB = xQueueCreate(10, sizeof(uint32_t));

long encoder1_Count = 0;
long encoder2_Count = 0;

volatile bool ENC2_CHBState;
volatile bool ENC1_CHBState;

void IRAM_ATTR ENC1_handler(void* args){
    ENC1_CHBState = digitalRead(ENC1_CHB);

    uint32_t state = ENC1_CHBState ? 1u : 0u;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(queueA, &state, &xHigherPriorityTaskWoken);
}

void IRAM_ATTR ENC2_handler(void* args){
    ENC2_CHBState = digitalRead(ENC2_CHB);

    uint32_t state = ENC2_CHBState ? 1u : 0u;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(queueB, &state, &xHigherPriorityTaskWoken);
}

void xENC1Task( void * pvParameters ){
    uint32_t ENC_CHB_State;

    for(;;){
        // Infinite loop to keep the task alive
        xQueueReceive(queueA, &ENC_CHB_State, portMAX_DELAY);
        if(ENC_CHB_State == 1){
            encoder1_Count++;
        } else {
            encoder1_Count--;
        }
        Serial.printf("Interrupt Triggered, Encoder 1 Count: %d\n" ,encoder1_Count);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void xENC2Task( void * pvParameters ){
    uint32_t ENC_CHB_State;

    for(;;){
        // Infinite loop to keep the task alive
        xQueueReceive(queueB, &ENC_CHB_State, portMAX_DELAY);
        if(ENC_CHB_State == 1){
            encoder2_Count--;
        } else {
            encoder2_Count++;
        }
        Serial.printf("Interrupt Triggered, Encoder 2 Count: %d\n" , encoder2_Count);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}


#endif

// Prototypes for Controller Callbacks
// Implementations located at the bottom of this file
void onConnection();
void onDisconnect();

void MainLoop();

TaskHandle_t LoopTaskHandle = NULL;
void LoopTask(void * parameter) {
  for(;;) {
    MainLoop();
  }
  vTaskDelay(1);
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
  if(ENCODER_EN){
    gpio_pad_select_gpio(ENC1_CHA);
    gpio_set_direction((gpio_num_t)ENC1_CHA, GPIO_MODE_INPUT);
    gpio_pulldown_en((gpio_num_t)ENC1_CHA);
    gpio_pullup_dis((gpio_num_t)ENC1_CHA);
    gpio_set_intr_type((gpio_num_t)ENC1_CHA, GPIO_INTR_POSEDGE);

    gpio_pad_select_gpio(ENC1_CHB);
    gpio_set_direction((gpio_num_t)ENC1_CHB, GPIO_MODE_INPUT);
    gpio_pulldown_en((gpio_num_t)ENC1_CHB);
    gpio_pullup_dis((gpio_num_t)ENC1_CHB);

    gpio_pad_select_gpio(ENC2_CHA);
    gpio_set_direction((gpio_num_t)ENC2_CHA, GPIO_MODE_INPUT);
    gpio_pulldown_en((gpio_num_t)ENC2_CHA);
    gpio_pullup_dis((gpio_num_t)ENC2_CHA);
    gpio_set_intr_type((gpio_num_t)ENC2_CHA, GPIO_INTR_POSEDGE);

    gpio_pad_select_gpio(ENC2_CHB);
    gpio_set_direction((gpio_num_t)ENC2_CHB, GPIO_MODE_INPUT);
    gpio_pulldown_en((gpio_num_t)ENC2_CHB);
    gpio_pullup_dis((gpio_num_t)ENC2_CHB);

    xTaskCreate(xENC1Task, "ENC1Task", 2048, nullptr, 2, &isrTaskHandle);
    xTaskCreate(xENC2Task, "ENC2Task", 2048, nullptr, 2, &isrTaskHandle);
    Serial.println("ISR Tasks created.");

    // Install GPIO ISR service and add handlers
    gpio_install_isr_service(0);
    gpio_isr_handler_add((gpio_num_t)ENC1_CHA, ENC1_handler, (void *) ENC1_CHA);
    gpio_isr_handler_add((gpio_num_t)ENC2_CHA, ENC2_handler, (void *) ENC2_CHA);
  }

  // Read robot info from "EEPROM" (ESP32 Preferences) using ConfigManager
  config.read();
  Serial.println(config.toString());
  robotType = config.getBotType();
  driveParams = config.getDriveParams();

  // work backwards from highest ordinal enum since lineman should be default case
  switch (robotType) {
    //* Each case should have the following:
    // An initialization of `robot` as a new Robot subclass
    // An initialization of `drive` as a new Drive subclass
    // A call to drive->setupMotors (or downcast and call to override)
    // An initialization of `lights` if needed depending on the bot type
    case kicker:
      robot = new Kicker(SPECBOT_PIN1, SPECBOT_PIN2, ENC1_CHA, ENC1_CHB);
      drive = new Drive(kicker, driveParams);
      drive->setupMotors(M1_PIN, M2_PIN);
      break;
    case quarterback_old:
      robot = new Quarterback(SPECBOT_PIN1, SPECBOT_PIN2, SPECBOT_PIN3);
      drive = new Drive(quarterback_old, driveParams);
      drive->setupMotors(M1_PIN, M2_PIN);
      break;
    case mecanum_center:
      robot = new MecanumCenter(SPECBOT_PIN1, SPECBOT_PIN2);
      drive = new DriveMecanum();
      ((DriveMecanum*) drive)->setupMotors(M1_PIN, M2_PIN, M3_PIN, M4_PIN);
      break;
    case center:
      robot = new Center(SPECBOT_PIN1, SPECBOT_PIN2);
      drive = new Drive(center, driveParams);
      drive->setupMotors(M1_PIN, M2_PIN);
      break;
    case runningback:
      robot = new Lineman();
      drive = new Drive(runningback, driveParams);
      drive->setupMotors(M1_PIN, M2_PIN);
      break;
    case quarterback_turret:
      robot = new QuarterbackTurret(
        M1_PIN, // left flywheel
        M2_PIN, // right flywheel
        M3_PIN, // cradle
        M4_PIN, // turret
        SPECBOT_PIN1, // assembly motor
        SPECBOT_PIN3, // magnetometer sda
        SPECBOT_PIN4, // magnetometer scl
        ENC1_CHA, // turret encoder
        ENC1_CHB, // turret encoder
        ENC2_CHB  // zeroing laser
      );
      break;
    case quarterback_base:
      drive = new Drive(quarterback_base, driveParams);
      drive->setupMotors(M1_PIN, M2_PIN);
      robot = new QuarterbackBase(drive);
      break;
    case receiver:
    case lineman:
    default: // Assume lineman
      robot = new Lineman();
      drive = new Drive(lineman, driveParams);
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

  ps5.attachOnConnect(onConnection);
  ps5.attachOnDisconnect(onDisconnect);
  xTaskCreatePinnedToCore(LoopTask, "LoopTask", 10000, NULL, 1, NULL, 1);
}

/*
   __  __      _      ___   _   _     _        ___     ___    ____
  |  \/  |    / \    |_ _| | \ | |   | |      / _ \   / _ \  |  _ \
  | |\/| |   / _ \    | |  |  \| |   | |     | | | | | | | | | |_) |
  | |  | |  / ___ \   | |  | |\  |   | |___  | |_| | | |_| | |  __/
  |_|  |_| /_/   \_\ |___| |_| \_|   |_____|  \___/   \___/  |_|

*/

// runs continuously after setup(). controls driving and any special robot functionality during a game
void MainLoop() {
  if (ps5.isConnected()) {
    // Serial.print(F("\r\nConnected"));
    // ps5.setLed(255, 0, 0);   // set LED red
    
    //* QBv3 Turret doesn't have drive, so this is a temporary measure to avoid NPEs and chaos
    // TODO: find better solution
    if (robotType != quarterback_turret) {
      if (robotType == mecanum_center) {
        ((DriveMecanum*) drive)->setStickPwr(ps5.LStickX(), ps5.LStickY(), ps5.RStickX());
      } else {
        drive->setStickPwr(ps5.LStickY(), ps5.RStickX());
      }

      // determine BSN percentage (boost, slow, or normal)
      if (ps5.Touchpad()){
        drive->emergencyStop();
        drive->setSpeedScalar(Drive::BRAKE);
      } else if (ps5.R1()) {
        drive->setSpeedScalar(Drive::BOOST);
        // ps5.setLed(0, 255, 0);   // set LED red
      } else if (ps5.L1()) {
        drive->setSpeedScalar(Drive::SLOW);
      } else if (ps5.R2() && (driveParams.motor_type == falcon || driveParams.motor_type == neo_vortex)) {
        // used to calibrate the max pwm signal for the falcon 500 motors
        drive->setSpeedValue(FALCON_CALIBRATION_FACTOR);
      } else {
        drive->setSpeedScalar(Drive::NORMAL);
      }

      if (ps5.Share()) 
        lights.setLEDStatus(Lights::DISCO);
      
      // Manual LED State Toggle (Home/Away/Off)
      if (ps5.Options()) 
        lights.togglePosition();
      
      // If the robot is able to hold the ball, it is able to be tackled:
      if (robotType == receiver || robotType == quarterback_old || robotType == runningback) {
        // if the lights are in the home or away state and the tackle pin goes low (tackle sensor is active low), enter the tackled state
        if ((lights.returnStatus() == Lights::HOME || lights.returnStatus() == Lights::AWAY) && digitalRead(TACKLE_PIN) == LOW) {
          lights.setLEDStatus(Lights::TACKLED);
          lights.tackleTime = millis();
        } 
        // leave the tackled state after some time and the tackle sensor pin went back to high
        else if ((millis() - lights.tackleTime) >= lights.switchTime && 
            lights.returnStatus() == Lights::TACKLED && digitalRead(TACKLE_PIN) == HIGH) {
          switch (lights.homeStatus()) {
            case Lights::HOME: lights.setLEDStatus(Lights::HOME); break;
            case Lights::AWAY: lights.setLEDStatus(Lights::AWAY); break;
            case Lights::OFF:  lights.setLEDStatus(Lights::OFF);  break;
          }
        }

        if (lights.returnStatus() == lights.DISCO)
          lights.updateLEDS();
      }
      //* Update the motors based on the inputs from the controller
      //* Can change functionality depending on subclass, like robot.action()
      drive->update();
      // drive->printDebugInfo(); // comment this line out to reduce compile time and memory usage
      // drive->printCsvInfo(); // prints info to serial monitor in a csv (comma separated value) format

      if (lights.returnStatus() == lights.DISCO && ((millis() - lights.updateTime) >= lights.updateSwitchTime)) {
        lights.updateLEDS();
        lights.updateTime = millis();
      }
    }
    //! Performs all special robot actions depending on the instantiated Robot subclass
    robot->action();

    // DEBUGGING:  
    // drive->printDebugInfo(); // comment this line out to reduce compile time and memory usage
    // drive->printCsvInfo(); // prints info to serial monitor in a csv (comma separated value) format
    // lights.printDebugInfo();

    // delay(5); // necessary for lights to be happy
      
  } else { // no response from PS5 controller within last 300 ms, so stop
    if (robotType != quarterback_turret) {
      // Emergency stop if the controller disconnects
      drive->emergencyStop();
      lights.setLEDStatus(Lights::UNPAIRED);
    } else {
      ((QuarterbackTurret*) robot)->emergencyStop();
    }
  }
}

void loop(){
  
}

/**
 * @brief onConnection: Function to be called on controller connect
 */
void onConnection() {
  if (ps5.isConnected()) {
    Serial.println(F("Controller Connected."));
    // ps5.setLed(0, 255, 0);   // set LED green
    lights.setLEDStatus(Lights::PAIRED);
  }

  // TODO: perm sln
  if (robotType != quarterback_turret) {
    drive->emergencyStop();
  } else {
    ((QuarterbackTurret*) robot)->emergencyStop();
  }
}

/**
 * @brief onDisconnect: Function to be called on controller disconnect
 * Stops bots from driving off and ramming into a wall or someone's foot if they disconnect
 */
void onDisconnect() {
    Serial.println(F("Controller Disconnected."));

    // TODO: perm sln
    if (robotType != quarterback_turret) {
      drive->emergencyStop();
    } else {
      ((QuarterbackTurret*) robot)->emergencyStop();
    }
}
