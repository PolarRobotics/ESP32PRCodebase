/*
  Question: Why are there so many #defines commented out?
  Answer: Documentation purposes.

  Question: Why not just use the #defines as they are written to generate the values?
  Answer: I don't trust the preprocessor. -MP
*/

//* Use https://blocks.jkniest.dev/ to generate comment block headers

#ifndef QUARTERBACK_TURRET_H
#define QUARTERBACK_TURRET_H

#include <Robot/Robot.h>
#include <Robot/MotorControl.h>
#include <ps5Controller.h>        // ESP PS5 library, access using global instance `ps5`
#include <Utilities/Debouncer.h>
#include <Adafruit_LIS3MDL.h>     // Magnetometer
#include <HardwareSerial.h>       // For ESP-to-ESP UART

enum TurretMode {
  manual, automatic, combine
};

enum TurretUnits {
  degrees, counts
};

enum AssemblyAngle {
  straight, angled, unknownAngle
};

enum CradleState {
  forward, back
};

enum TargetReceiver {
  receiver_1, receiver_2
};

enum CombinePosition {
  combineLeft, combineStraight, combineRight
};

enum FlywheelSpeed {
  slow_inwards, stopped, slow_outwards, lvl1_outwards, lvl2_outwards, lvl3_outwards, maximum
};
#define QB_TURRET_NUM_SPEEDS 7
// const float flywheelSpeeds[QB_TURRET_NUM_SPEEDS] = {-0.1, 0, 0.1, 0.225, 0.35, 0.45, 1.0}; // with top prongs
const float flywheelSpeeds[QB_TURRET_NUM_SPEEDS] = {-0.1, 0, 0.1, 0.215, 0.31, 0.3875, 1.0}; // without top prongs

//================================//
//  Debounce and Delay Constants  //
//================================//
// 50 ms for default delay (50L)
// 750 ms to fully extend or retract the linear actuator
#define QB_BASE_DEBOUNCE_DELAY 50L
#define QB_ASSEMBLY_TILT_DELAY 200L
#define QB_CRADLE_TRAVEL_DELAY 750L
#define QB_CIRCLE_HOLD_DELAY 750L
#define QB_TRIANGLE_HOLD_DELAY 200L
#define QB_CROSS_HOLD_DELAY 200L
#define QB_TURRET_INTERPOLATION_DELAY 5L
#define QB_TURRET_THRESHOLD 35
#define QB_TURRET_STICK_SCALE_FACTOR 0.15 // was 0.25, turned down for combine

//================================//
//        Speed Constants         //
//================================//
#define QB_MIN_PWM_VALUE 0.1
#define QB_HOME_PCT 0.125
#define QB_HANDOFF  0.3
#define QB_HOME_MAG 0.1
#define QB_ASM_SPEED 0.3

//========================================//
//   Turret Angle Calculation Constants   //
//========================================//
// QB_COUNTS_PER_ENCODER_REV      Number of ticks per encoder revolution
// QB_COUNTS_PER_TURRET_REV       Encoder Gear: 18t; Turret Track: 152t. Encoder spins 8.4 times for every turret revolution = 8500 ticks/rev
// QB_COUNTS_PER_TURRET_DEGREE    (8500/360) = 23.61... ticks per degree
// QB_TURRET_SLOP_COUNTS          Backlash between input and output on the turret is high -> leads to problems when switching directions
//                                    540 ticks on encoder before turret actually starts to move (Empirically measured)
#define QB_COUNTS_PER_ENCODER_REV 1250
#define QB_COUNTS_PER_TURRET_REV 10556 // actually 10555.55555...6, but rounded up
#define QB_COUNTS_PER_TURRET_DEGREE 29.321
#define QB_TURRET_SLOP_COUNTS 540 // deprecated
// #define QB_FALCON_TO_TURRET_RATIO 27 / 1
// #define QB_ENCODER_TO_FALCON_RATIO 5 / 1
// #define QB_ENCODER_TO_TURRET_RATIO  /* = */ QB_FALCON_TO_TURRET_RATIO / QB_ENCODER_TO_FALCON_RATIO // (27 / 5) = 5.4
// #define QB_COUNTS_PER_TURRET_REV    /* = */ QB_ENCODER_TO_TURRET_RATIO * QB_COUNTS_PER_ENCODER_REV // 5400
// #define QB_COUNTS_PER_TURRET_DEGREE /* = */ QB_COUNTS_PER_TURRET_REV / 360
// #define QB_TURRET_SLOP_GEAR_TEETH   /* = */ 10 // slop is about 10 gear teeth
// #define QB_TURRET_SLOP_PCT          /* = */ QB_DIRECTION_CHANGE_SLOP_GEAR_TEETH / 100
// #define QB_TURRET_SLOP_COUNTS       /* = */ QB_DIRECTION_CHANGE_SLOP_PCT * QB_COUNTS_PER_TURRET_REV

//===============================//
//    Turret Homing Constants    //
//===============================//
// QB_TURRET_STOP_LOOP_DELAY_MS         
// QB_TURRET_STOP_THRESHOLD_MS          must be a multiple of QB_TURRET_STOP_LOOP_DELAY_MS
// QB_TURRET_HOME_STOP_FACTOR           correction constant for homing, multiplied into stop counts in final homing
// QB_TURRET_MANUAL_CONTROL_FACTOR      higher values = less sensitive during manual control (Basically divides the clock of the loop)
#define QB_TURRET_STOP_LOOP_DELAY_MS 10
#define QB_TURRET_STOP_THRESHOLD_MS 500
#define QB_TURRET_HOME_STOP_FACTOR 0 // 0.5
#define QB_TURRET_MANUAL_CONTROL_FACTOR 4 

//=====================================//
//   Turret PID Controller Constants   //
//=====================================//
// QB_TURRET_PID_THRESHOLD      Acceptable error in position control (degrees) that will zero the error constants / build up
// QB_TURRET_PID_MIN_DELTA_T    If PID error values are not updated fast enough, for example if the controller hangs up for half a second,
//                                  then the PWM values will be massive compared to what they should be so just kind of reset everything
// QB_TURRET_PID_MAX_DELTA_T    Maximum time between updates
// QB_TURRET_PID_BAD_DELTA_T    Maximum time before we need to zero things out
// QB_NORTH_OFFSET              Added to deal with the problems of zeroing the turret but then holding a set angle afterwards
//                                  TODO: maybe unnecessary now?
#define QB_TURRET_PID_THRESHOLD 3
#define QB_TURRET_PID_MIN_DELTA_T 5
#define QB_TURRET_PID_MAX_DELTA_T 25
#define QB_TURRET_PID_BAD_DELTA_T 250
#define QB_NORTH_OFFSET 0

// Enable or Disable Auto Mode for testing
#define QB_AUTO_ENABLED false

//===============================//
//    UART Communication Pins    //
//===============================//
// TODO: move to appropriate area and pass into constructor
#define RX2 16 // reciever pin
#define TX2 17 // transmitter pin

/**
 * @brief Quarterback Turret Subclass Header
 * @authors Maxwell Phillips, George Rak
 */
class QuarterbackTurret : public Robot {

#pragma region Private
  //============================//
  //|                          |//
  //|          PRIVATE         |//
  //|                          |//
  //============================//
  private: 

    //==============================//
    //    MotorControl Instances    //
    //==============================//
    MotorControl cradleActuator;
    MotorControl turretMotor;
    MotorControl assemblyMotor; 
    MotorControl flywheelLeftMotor;
    MotorControl flywheelRightMotor;
    
    //==============================//
    //       Pin Declarations       //
    //==============================//
    static uint8_t turretEncoderPinA;
    static uint8_t turretEncoderPinB;
    uint8_t turretLaserPin;

    //===============================//
    //        Joystick Inputs        //
    //===============================//
    // stickTurret    used to normalize stick input from [0, 255] to [-1.0, 1.0]
    // stickFlywheel  same as above  
    float stickTurret;
    float stickFlywheel;

    //==============================//
    //     Autonomous Targeting     //
    //==============================//
    // mode: manual or autonomous (or combine)
    // target: reciever1 or reciever2
    TurretMode mode;
    TargetReceiver target;
    CombinePosition combinePosition;

    //==============================//
    //  Setup and Status Variables  //
    //==============================//
    // enabled        default false, set true when homing / reset complete (toggleable via function)
    // initialized    default false, set true when homing / reset complete (stays true) -> 
    //                    decides whether or not to just run zero turret or entire setup routine
    // runningMacro   default false, set true while a macro is running
    bool enabled; 
    bool initialized; 
    bool runningMacro;

    //===============================//
    //       Assembly Movement       //
    //===============================//
    // currentAssemblyAngle   initial state is unknown, will reset to [straight]. options: [straight, angled]. 
    // targetAssemblyAngle    defaults to [straight]. options: [straight, angled]. 
    // assemblyMoving         defaults to false
    AssemblyAngle currentAssemblyAngle;
    AssemblyAngle targetAssemblyAngle;
    bool assemblyMoving;
    uint32_t assemblyStartTime;
    bool assemblyTriggerToggled;

    //===============================//
    //  Ball Cradle State Variables  //
    //===============================//
    // currentCradleState   initial state is unknown, will reset to [back]. options: [forward, back].
    // targetCradleState    defaults to [back]. options: [forward, back].
    // cradleMoving         defaults to [false]
    // cradleStartTime      
    CradleState currentCradleState;
    CradleState targetCradleState;
    bool cradleMoving;
    uint32_t cradleStartTime;


    //===============================//
    //        Flywheel State         //
    //===============================//
    // currentFlywheelStage       defaults to stopped
    // targetFlywheelStage        defaults to stopped
    // currentFlywheelSpeed       defaults to 0
    // targetFlywheelsSpeed       defaults to 0
    // flywheelManualOverride     defaults to false, true when stick is controlling flywheel
    FlywheelSpeed currentFlywheelStage;
    FlywheelSpeed targetFlywheelStage;
    float currentFlywheelSpeed;
    float targetFlywheelSpeed;
    bool flywheelManualOverride;

    //===============================//
    //         Turret State          //
    //===============================//
    // currentTurretSpeed   default = 0
    // targetTurretSpeed    default = 0
    float currentTurretSpeed;
    float targetTurretSpeed;
    int8_t utmsCtr = 0; // temp ctr for debugging so updateTurretMotionStatus doesn't spam as much
    #define UTMS_CTR_MAX 15

    //=====================================//
    //   Private Encoder State Variables   //
    //=====================================//
    // * see also header [[Public Encoder State Variables for ISR]]
    // targetTurretEncoderCount       default = 0. complementary to [currentTurretEncoderCount]
    // errorEncoderCount              the error [currentTurretEncoderCount - targetTurretEncoderCount]
    // slopError                      any error due to mechanical backlash in the gears, set by robot during homing / reset
    // stopError                      number of encoder counts needed for motor to stop moving
    // turretMoving                   set to true when the turret is moving asynchronously or in the normal program
    // manualHeadingIncrementCount    default = 0
    int32_t targetTurretEncoderCount;
    int32_t errorEncoderCount;
    int32_t slopError;                // TODO: remove with new encoder
    int32_t stopError;                // TODO: remove with new encoder (?)
    bool turretMoving;
    uint8_t manualHeadingIncrementCount;

    //===============================//
    //    Robot Relative Headings    //
    //===============================//     
    int16_t currentRelativeHeading;       // default is undefined
    int16_t targetRelativeHeading;        // default = 0 //* as of 2024-11-15, in degrees only
    int32_t currentRelativeTurretCount;   // default is undefined
    
    //========================================//
    //   Absolute (World Relative) Headings   //
    //========================================//
    // currentAbsoluteHeading         
    // targetAbsoluteHeading          
    // turretLaserState               
    int16_t currentAbsoluteHeading;   // default = 0
                                      // TODO: is unused, merge with headingRad/headingDeg
    int16_t targetAbsoluteHeading;    // default = 0
    uint8_t turretLaserState;         // triggered or not 
                                      // TODO: seems to be unused, remove?

    //================================//
    //    Control Input Debouncers    //
    //================================//
    Debouncer* dbShare;
    Debouncer* dbOptions;
    Debouncer* dbSquare;
    Debouncer* dbDpadUp;
    Debouncer* dbDpadDown;
    Debouncer* dbDpadLeft;
    Debouncer* dbDpadRight;

    //===============================//
    //    Macro Button Debouncers    //
    //===============================//
    //* used to set a delay that a button must be held in order to trigger a macro
    Debouncer* dbCircle;
    Debouncer* dbTriangle;
    Debouncer* dbCross;
    Debouncer* dbTurretInterpolator; // TODO: seems to be unused, remove?

#pragma region Magnetometer
    //==========================//
    //|                        |//
    //|      Magnetometer      |//
    //|                        |//
    //==========================//
    Adafruit_LIS3MDL lis3mdl;           // magnetometer object
    bool useMagnetometer = true;        // set 'false' to disable the magnetometer and its functions
    bool holdTurretStillEnabled = true; // set 'false' if you only want to use the magnetometer for the handoff and not the hold steady

    //============================//
    //  Magnetometer Calibration  //
    //============================//
    //* used at each startup
    // TODO: make a class or struct for all these
    // mag_xVal, mag_yVal:     The current x and y values read by the magnetometer (after adjustments)
    // mag_xMax, mag_xMin:     During calibration the max and min X values are recorded
    // mag_yMax, mag_yMin:     During calibration the max and min Y values are recorded
    // mag_xHalf, mag_yHalf:   Half of the total range of expected x and Y values (Used to shift values back to origin [0,0])
    // mag_xSign, mag_ySign:   Determines whether to add to the half value or subtract from it when shifting values back to origin (true = subtract)
    int mag_yVal = 0;
    int mag_xVal = 0;
    int mag_xMax = -1000000;
    int mag_xMin = 1000000;
    int mag_xHalf = 0;
    int mag_yMax = -1000000;
    int mag_yMin = 1000000;
    int mag_yHalf = 0;
    bool mag_xSign = false;
    bool mag_ySign = false;
    int northHeadingDegrees = 0;


    /* Magnetometer current heading calculations
        - headingRad:     The current calculated heading in radians using the X and Y values after calibration
        - headingDeg:     The current calculated heading in degrees -> uses headingRad
    */
    float headingRad; // TODO: merge with robot abs pot vars
    float headingDeg; // TODO: merge with robot abs pot vars

    //==================================//
    //    Magnetometer PID Variables    //
    //==================================//
    // PID_ERROR_AVG_ARRAY_LENGTH       The length of the array used for averaging the error

    // prevErrorVals        Array of previous error values used to average the last few error values together
    //                          to smooth out random spikes in readings from magnetometer
    // prevErrorIndex       The current index gets replaced with the new error value cycling through the entire array over time
    // firstAverage         On the first error calculations, the array is filled with zeros. 
    //                          This accounts for that by filling the entire array with the current reading.

    // previousTime:        Used to calculate errors and deltaT in PID function
    // ePrevious:           The error builds as time goes on, this is used to record the previous and new error values
    // eIntegral:           The current error integral calculated, will be added to ePrevious in PID loop
    // kp:                  Proportional gain used in PID
    // ki:                  Integral gain used in PID
    // kd:                  Derivative gain used in PID
    // turretPIDSpeed:      The calculated PWM value used in the PID loop
    // minMagSpeed:         Small PWM signals fail to make the motor turn leading to error in PID calculations. 
    //                        This sets a bottom bound on the PWM signal that can be calculated by the PID loop
    #define PID_ERROR_AVG_ARRAY_LENGTH 5
    // TODO: convert to appropriate (u)int#_t types
    int prevErrorVals[PID_ERROR_AVG_ARRAY_LENGTH] = { 0, 0, 0, 0, 0 };
    int prevErrorIndex = 0;
    bool firstAverage = true;
    long previousTime = 0;
    float ePrevious = 0;
    float eIntegral = 0;
    float kp = 0.005;
    float ki = 0.0012;
    float kd = 0.0;
    float turretPIDSpeed = 0;
    float minMagSpeed = .075;

    //============================//
    //   Magnetometer Functions   //
    //============================//
    // magnetometerSetup      Sets some of the parameters. Runs once on startup.
    // calibMagnetometer      Rotates turret once on startup, records readings and sets values that will scale/transform 
    //                            outputs to the correct angles
    // calculateHeadingMag    Uses the values calculated during calibration to find the current heading of the turret 
    //                            with respect to the original 0 position
    // holdTurretStill        Checks if the calibration has been completed and hold turret stil is enabled then enables the PID loop
    void magnetometerSetup();
    void calibMagnetometer();
    void calculateHeadingMag();
    void holdTurretStill();
    float turretPIDController(float current, float target, float kp, float kd, float ki, float maxSpeed);

    /* MAGNETOMETER CURRENT STATE NOTES / PLAN (from April 9th, 2024 7:56 PM)
      - the PID controller works pretty well, tested on table rotating quickly
      - Tested PID controller on field and it immidiately flipped so we need to tune it so that the robot does not tip itself over
      - Magnetometer mount needs redesigned and printed to be more stable / protective of the magnetometer and wires 
          (Should probably solder wires to magnetometer for best reliability)
      - Ability to change holding angle of turret with joystick needs evaluated for correctness
      - Turret flywheel equation needs generated for requested distance
      - Scan and Score capstone integration needs done to control angle and throw distance
      - Testing needs done to see how much flywheels beign on affects magnetometer
      - Relative velocities should be taken into account with trajectory calculations
    */
#pragma endregion

    //====================================//
    //     Private UART Communication     //
    //====================================//
    String recievedMessage = "";

    //=============================//
    //   Misc. Private Functions   //
    //=============================//
    void moveAssemblySubroutine();
    void moveCradleSubroutine();
    void moveTurret(int16_t heading, TurretUnits units, float power = QB_HOME_PCT, bool relativeToRobot = true, bool ramp = false); 
    void updateTurretMotionStatus();
    int16_t getCurrentHeading();
    int16_t findNearestHeading(int16_t targetHeading, int16_t currentHeading);
    int16_t findNearestHeading(int16_t targetHeading);
    int NormalizeAngle(int angle);
    int CalculateRotation(float currentAngle, float targetAngle);
#pragma endregion

#pragma region Public
  //===========================//
  //|                         |//
  //|          PUBLIC         |//
  //|                         |//
  //===========================//
  public:
    //========================//
    //      Constructor       //
    //========================//
    QuarterbackTurret(
      uint8_t flywheelLeftPin,    // M1
      uint8_t flywheelRightPin,   // M2
      uint8_t cradlePin,          // M3
      uint8_t turretPin,          // M4
      uint8_t assemblyPin,        // S1
      uint8_t magnetometerSdaPin, // S3
      uint8_t magnetometerSclPin, // S4
      uint8_t turretEncoderPinA,  // E1A
      uint8_t turretEncoderPinB,  // E1B
      uint8_t turretLaserPin      // E2A
    );

    bool magnetometerCalibrated = false;

    //===================================//
    //   Quarterback General Functions   //
    //===================================//
    // action                   robot sublass must override action    
    void action() override;

    // [Startup]
    // zeroTurret               Runs through the routine to zero the turret
    // reset                    zero turret, aim down (straight), and set flywheels to slow intake
    void zeroTurret();
    void reset();

    // [Safety]
    // setEnabled               toggles turret and flywheel movement
    // emergencyStop            If the E-Stop button on controller is pressed this should override every other motor function and force them to stop
    // testForDisableOrStop     Used for safety, returns false if the robot is disabled or should be stopped
    void setEnabled(bool enabled);
    void emergencyStop();
    bool testForDisableOrStop();

    // [Debug]
    // printDebug               Consolidating printouts for the different functions into one locations for debugging purposes
    // testRoutine              Used for debugging
    void printDebug();
    void testRoutine();


    //====================================//
    //   Quarterback Subsystem Controls   //
    //====================================//
    // setTurretSpeed                 Moves the turret at a specified speed (open loop)
    // moveTurret                     Turn the turret to a specific heading (currently relative to robot) -> asynchronous
    // moveTurretAndWait              Moves the turret and waits until the heading is reached -> synchronous
    // aimAssembly                    Moves the falcon pylon assembly up and down between two states
    // moveCradle                     Moves the cradle between two states throw and hold
    // setFlywheelSpeed               Sets the speed of the flywheels using stick inputs
    // setFlywheelSpeedStage          Set the speed as one of the defined enums
    // adjustFlywheelSpeedStage       Move to the next level up or down in the list of speeds
    void setTurretSpeed(float absoluteSpeed, bool overrideEncoderTare = false); 
    void moveTurret(int16_t heading, bool relativeToRobot = true, bool ramp = true); 
    void moveTurret(int16_t heading, float power = QB_HOME_PCT, bool relativeToRobot = true, bool ramp = true);
    void moveTurretAndWait(int16_t heading, float power = QB_HOME_PCT, bool relativeToRobot = true, bool ramp = true);
    void aimAssembly(AssemblyAngle angle, bool force = false); 
    void moveCradle(CradleState state, bool force = false); 
    void setFlywheelSpeed(float absoluteSpeed); 
    void setFlywheelSpeedStage(FlywheelSpeed stage); 
    void adjustFlywheelSpeedStage(SpeedStatus speed);

    //===================================//
    //  Quarterback Automatic Targeting  //
    //===================================//
    // switchMode                 Used to switch between manual and automatic mode (overloaded function)
    // switchTarget               Switch between receiver1 or receiver2
    void switchMode(TurretMode mode); 
    void switchMode(); 
    void switchTarget(TargetReceiver target); 

    //==================================//
    //   Quarterback Strategic Macros   //
    //==================================//
    // loadFromCenter             Prepares the QB to accept the ball from the center
    // handoff                    Hands the ball to the runningback
    void loadFromCenter();
    void handoff();
    
    
    //==========================================//
    //  Public Encoder State Variables for ISR  //
    //==========================================//
    // * see also header [[Private Encoder State Variables]]
    // * any variables used in the turretEncoderISR() (interrupt service routine) must be public static
    static int32_t currentTurretEncoderCount;   // updated by encoder ISR (interrupt service routine). complementary to [targetTurretEncoderCount].
    static uint8_t turretEncoderStateB;         // A channel will always be 1 when interrupt triggers, so only care about channel B

    //===============================================//
    //    Encoder Interrupt Service Routine (ISR)    //
    //===============================================//
    // must be static. simple function called when encoder interrupts are triggered. updates [currentTurretEncoderCount].
    static void turretEncoderISR(); 

    // If the direction changes then some things need to happen differently because of the incredible amount of backlash
    // TODO: remove? may not be needed with new encoder system
    void turretDirectionChanged(); 

    //===================================//
    //     Public UART Communication     //
    //===================================//
    int motor1Value = 0;
    int motor2Value = 0;
    void updateReadMotorValues();

#pragma endregion
};  

#endif // QUARTERBACK_TURRET_H
