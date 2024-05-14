/*Max Statements
  Question: Why are there so many #defines commented out?
  Answer: Documentation purposes.

  Question: Why not just use the #defines as they are written to generate the values?
  Answer: I don't trust the preprocessor. -MP

*/

#pragma once

#ifndef QUARTERBACK_TURRET_H
#define QUARTERBACK_TURRET_H

#include <Robot/Robot.h>
#include <Robot/MotorControl.h>
#include <ps5Controller.h> // ESP PS5 library, access using global instance `ps5`
#include <Utilities/Debouncer.h>
#include <Adafruit_LIS3MDL.h>
#include <HardwareSerial.h>

enum TurretMode {
  manual, automatic
};

enum TurretUnits {
  degrees, counts
};

enum AssemblyAngle {
  straight, angled
};

enum CradleState {
  forward, back
};

enum TargetReceiver {
  receiver_1, receiver_2
};

enum FlywheelSpeed {
  slow_inwards, stopped, slow_outwards, lvl1_outwards, lvl2_outwards, lvl3_outwards, maximum
};
#define QB_TURRET_NUM_SPEEDS 7
const float flywheelSpeeds[QB_TURRET_NUM_SPEEDS] = {-0.1, 0, 0.1, 0.3, 0.5, 0.7, 1.0};

/*DEBOUNCE & OTHER DELAY CONSTANTS
  - 50 ms for default delay (50L)
  - 750 ms to fully extend or retract the linear actuator
*/
#define QB_BASE_DEBOUNCE_DELAY 50L
#define QB_CRADLE_TRAVEL_DELAY 750L
#define QB_CIRCLE_HOLD_DELAY 750L
#define QB_TRIANGLE_HOLD_DELAY 200L
#define QB_CROSS_HOLD_DELAY 200L
#define QB_TURRET_INTERPOLATION_DELAY 5L
#define QB_TURRET_THRESHOLD 35
#define QB_TURRET_STICK_SCALE_FACTOR 0.25

/*SPEED CONSTANTS*/
#define QB_MIN_PWM_VALUE 0.08
#define QB_HOME_PCT 0.125
#define QB_HANDOFF  0.3
#define QB_HOME_MAG 0.1

/*TURRET ANGLE CALCULATION CONSTANTS
  - QB_COUNTS_PER_ENCODER_REV       Number of ticks per encoder revolution
  - QB_COUNTS_PER_TURRET_REV        27:1 falcon to turret | 5:1 falcon to encoder (12t driving sprocket on 60t gear) | Encoder spins 5.4 times (27/5) for every turret revolution = 5400 ticks per rev
  - QB_COUNTS_PER_TURRET_DEGREE     (5400/360) = 15 ticks per degree
  - QB_TURRET_SLOP_COUNTS           Backlash between input and output on the turret is high -> leads to problems when switching directions = 540 ticks on encoder before turret actually starts to move (Empirically measured)
*/
#define QB_COUNTS_PER_ENCODER_REV 1000
#define QB_COUNTS_PER_TURRET_REV 5400
#define QB_COUNTS_PER_TURRET_DEGREE 15
#define QB_TURRET_SLOP_COUNTS 540
// #define QB_FALCON_TO_TURRET_RATIO 27 / 1
// #define QB_ENCODER_TO_FALCON_RATIO 5 / 1
// #define QB_ENCODER_TO_TURRET_RATIO /* = */ QB_FALCON_TO_TURRET_RATIO / QB_ENCODER_TO_FALCON_RATIO // (27 / 5) = 5.4
// #define QB_COUNTS_PER_TURRET_REV /* = */ QB_ENCODER_TO_TURRET_RATIO * QB_COUNTS_PER_ENCODER_REV // 5400
// #define QB_COUNTS_PER_TURRET_DEGREE /* = */ QB_COUNTS_PER_TURRET_REV / 360
// #define QB_TURRET_SLOP_GEAR_TEETH /* = */ 10 // slop is about 10 gear teeth
// #define QB_TURRET_SLOP_PCT        /* = */ QB_DIRECTION_CHANGE_SLOP_GEAR_TEETH / 100
// #define QB_TURRET_SLOP_COUNTS     /* = */ QB_DIRECTION_CHANGE_SLOP_PCT * QB_COUNTS_PER_TURRET_REV

/*TURRET HOMING CONSTANTS
  -QB_TURRET_STOP_LOOP_DELAY_MS 
  -QB_TURRET_STOP_THRESHOLD_MS          must be a multiple of QB_TURRET_STOP_LOOP_DELAY_MS
  -QB_TURRET_HOME_STOP_FACTOR           correction constant for homing, multiplied into stop counts in final homing
  -QB_TURRET_MANUAL_CONTROL_FACTOR      higher values = less sensitive during manual control (Basically divides the clock of the loop)
*/
#define QB_TURRET_STOP_LOOP_DELAY_MS 10
#define QB_TURRET_STOP_THRESHOLD_MS 500
#define QB_TURRET_HOME_STOP_FACTOR 0.5
#define QB_TURRET_MANUAL_CONTROL_FACTOR 4 

/*TURRET PID CONTROLLER CONSTANTS
  -QB_TURRET_PID_THRESHOLD              Acceptable error in position control (degrees) that will zero the error constants / build up
  -QB_TURRET_PID_MIN_DELTA_T            If PID error values are not updated fast enough example the controller hangs up for half a second then the PWM values will be massive compared to what they should be so just kind of reset everything
  -QB_TURRET_PID_MAX_DELTA_T            Maximum time between updates
  -QB_TURRET_PID_BAD_DELTA_T            Maximum time before we need to zero things out
  -QB_NORTH_OFFSET                      Added to deal with the problems of zeroing the turret but then holding a set angle afterwards
*/
#define QB_TURRET_PID_THRESHOLD 3
#define QB_TURRET_PID_MIN_DELTA_T 5
#define QB_TURRET_PID_MAX_DELTA_T 25
#define QB_TURRET_PID_BAD_DELTA_T 250
#define QB_NORTH_OFFSET 0

// Enable or Disable Auto Mode for testing
#define QB_AUTO_ENABLED false

/*UART COMMUNICATION PINS
  -RX2          Used to Recieve data
  -TX2          Used to Transmit data
*/
#define RX2 16
#define TX2 17


/**
 * @brief Quarterback Turret Subclass Header
 * @authors Maxwell Phillips
 */
class QuarterbackTurret : public Robot {
  private: 

    /*MOTOR INSTANCES*/
    MotorControl cradleActuator;
    MotorControl turretMotor;
    MotorControl flywheelLeftMotor;
    MotorControl flywheelRightMotor;
    
    /*PIN DECLARATIONS (PERSISTENT)*/
    static uint8_t turretEncoderPinA;
    static uint8_t turretEncoderPinB;
    uint8_t turretLaserPin;

    /*JOYSTICK INPUTS
      -stickTurret          used to normalize stick input from [0, 255] to [-1.0, 1.0]
      -stickFlywheel        same as above
    */
    float stickTurret;
    float stickFlywheel;

    /*MODES AND CAPSTONE INTEGRATIONS
      -mode         default manual
      -target       default receiver_1
    */
    TurretMode mode;
    TargetReceiver target;

    /*MSC VARIABLES
      -enabled              default false, set true when homing / reset complete (toggleable via function)
      -initialized          default false, set true when homing / reset complete (stays true) -> decides whether or not to just run zero turret or entire setup routine
      -runningMacro         default false, set true while a macro is running
    */
    bool enabled; 
    bool initialized; 
    bool runningMacro;

    /*FALCON PYLONS
      -currentAssemblyAngle       initial state is unknown, will reset to straight
      -targetAssemblyAngle        defaults to straight
      -assemblyMoving             defaults to false
    */
    AssemblyAngle currentAssemblyAngle;
    AssemblyAngle targetAssemblyAngle;
    bool assemblyMoving;

    /*CRADEL
      -currentCradleState         initial state is unknown, will reset to back
      -targetCradleState          defaults to back
      -cradleMoving               defaults to false
      -cradleStartTime            
    */
    CradleState currentCradleState;
    CradleState targetCradleState;
    bool cradleMoving;
    uint32_t cradleStartTime;


    /*FLYWHEEL
      -currentFlywheelStage       defaults to stopped
      -targetFlywheelStage        defaults to stopped
      -currentFlywheelSpeed       defaults to 0
      -targetFlywheelsSpeed       defaults to 0
      -flywheelManualOverride     defaults to false, true when stick is controlling flywheel
    */
    FlywheelSpeed currentFlywheelStage;
    FlywheelSpeed targetFlywheelStage;
    float currentFlywheelSpeed;
    float targetFlywheelSpeed;
    bool flywheelManualOverride;

    /*TURRET
      -currentTurretSpeed         default = 0
      -targetTurretSpeed          default = 0
    */
    float currentTurretSpeed;
    float targetTurretSpeed;

    /*ROBOT RELATIVE HEADINGS
      -currentRelativeHeading           default is undefined
      -targetrelativeHeading            default = 0
      -currentRelativeTurretCount       default is undefined
      -targetTurretEncoderCount         default = 0
      -errorEncoderCount                the error (current - target)
      -slopError                        any error due to mechanical backlash in the gears, set by robot during homing / reset
      -stopError                        number of encoder counts needed for motor to stop moving
      -turretMoving                     set to true when the turret is moving asynchronously or in the normal program
      -manualHeadingIncrementCount      default = 0
    */
    int16_t currentRelativeHeading;
    int16_t targetRelativeHeading;
    int32_t currentRelativeTurretCount;
    int32_t targetTurretEncoderCount;
    int32_t errorEncoderCount;
    int32_t slopError;
    int32_t stopError;
    bool turretMoving;
    uint8_t manualHeadingIncrementCount;

    /*WORLD RELATIVE HEADINGS
      -currentAbsoluteHeading       default = 0
      -targetAbsoluteHeading        default = 0
      -turretLaserState             triggered or not
    */
    int16_t currentAbsoluteHeading;
    int16_t targetAbsoluteHeading;
    uint8_t turretLaserState;

    /*DEBOUNCERS*/
    Debouncer* dbOptions;
    Debouncer* dbSquare;
    Debouncer* dbDpadUp;
    Debouncer* dbDpadDown;

    //for delay
    Debouncer* dbCircle;
    Debouncer* dbTriangle;
    Debouncer* dbCross;
    Debouncer* dbTurretInterpolator;

    /******* MAGNETOMETER ******/
    Adafruit_LIS3MDL lis3mdl;
    bool useMagnetometer = true;  //Change this if you want to use the magnetometer or any of it's functions
    bool holdTurretStillEnabled = true; //Change this if you only want to use the magnetometer for the handoff and not the hold steady

    /* Magnetometer calibration variables used at startup each time
        - yVal, xVal:     The current x and y values read by the magnetometer (after adjustments)
        - maxX, minX:     During calibration the max and min X values are recorded
        - maxY, minY:     During calibration the max and min Y values are recorded
        - xHalf, yHalf:   Half of the total range of expected x and Y values (Usesd to shift values back to origin [0,0])
        - xsign, ysign:   Determines whether to add to the half value or subtract from it when shifting values back to origin (true = subtract)
    */
    int yVal = 0;
    int xVal = 0;
    int maxX = -1000000;
    int minX = 1000000;
    int xHalf = 0;
    int maxY = -1000000;
    int minY = 1000000;
    int yHalf = 0;
    bool xsign = false;
    bool ysign = false;
    int northHeadingDegrees = 0;

    /* Magnetometer current heading calculations
        - headingrad:     The current calculated heading in radians using the X and Y values after calibration
        - headingdeg:     The current calculated headign in degrees -> uses headingrad
    */
    float headingrad;
    float headingdeg;

    /* Magnetometer PID calculations
        - previousTime:         Used to calcualte errors and deltaT in PID function
        - ePrevious:            The error builds as time goes on, this is used to record the previous and new error values
        - eIntegral:            The current error integral calculated, will be added to ePrevious in PID loop
        - kp:                   Proportional gain used in PID
        - ki:                   Integral gain used in PID
        - kd:                   Derivative gain used in PID
        - turretPIDSpeed:       The calculated PWM value used in the PID loop
        - prevErrorVals:        Array of previous error values used to average the last few together to smooth out random spikes in readings from magnetometer
        - prevErrorIndex:       The current index gets replaced with the new error value cycling through the entire array over time
        - errorAverageLength:   The length of the array used for averaging the error
        - firstAverage:         On the first error calculations the array is filled with zeros so this accounts for that by filling the entire array with the current reading
        - minMagSpeed:          Small PWM signals fail to make the motor turn leading to error in PID calculations, this sets a bottom bound on the PWM signal that can be calculated by the PID loop
    */
    long previousTime = 0;
    float ePrevious = 0;
    float eIntegral = 0;
    float kp = .005;
    float ki = 0.0012;
    float kd = 0.0;
    float turretPIDSpeed = 0;
    int prevErrorVals[5] = {0,0,0,0,0};
    int prevErrorIndex = 0;
    int errorAverageLength = 5;
    bool firstAverage = true;
    float minMagSpeed = .075;

    /* Magnetometer functions:
        - magnetometerSetup       Sets some of the parameters runs once
        - calibMagnetometer       Rotates turret once on startup recording readings and setting values that will move and scale outputs to be correct angle
        - calculateHeadingMag     Uses the values calculated during calibration to find the current heading of the turret with respect to the original 0 position
        - holdTurretStill         Checks if the calibration has been completed and hold turret stil is enabled then enables the PID loop
        - turretPIDController     PID loop that drives turret to the current setpoint
    */
    void magnetometerSetup();
    void calibMagnetometer();
    void calculateHeadingMag();
    void holdTurretStill();
    float turretPIDController(int setPoint, float kp, float kd, float ki, float maxSpeed);

    /* MAGNETOMETER CURRENT STATE NOTES / PLAN
      - the PID controller works pretty well, tested on table rotating quickly
      - Tested PID controller on field and it immidiately flipped so we need to tune it so that the robot does not tip itself over
      - Magnetometer mount needs redesigned and printed to be more stable / protective of the magnetometer and wires (Should probably solder wires to magnetometer for best reliability)
      - Ability to change holding angle of turret with joystick needs evaluated for correctness
      - Turret flywheel equation needs generated for requested distance
      - Nathan capstone integration needs done to control angle and thorw distance
      - Testing needs done to see how much flywheels beign on affects magnetometer
      - Relative velocities should be taken into account with trajectory calculations
    */

    /*UART VARIABLES*/
    String recievedMessage = "";


    /*MSC PRIVATE FUNCTION DECLARATIONS
      -moveCradleSubroutine       Avoids code duplication on ball fondler
      -moveTurret                 Moves turret / turntable to specific heading (relative to robot not the field)
      -updateTurretMotionStatus   Allows for asynchronous turret movements (Stops when turret reaches target position)
      -getCurrentHeading          Calculates current heading from the current encoder count
      -findNearestHeading         Finds the smallest absolute value difference from either the current or provided angle (overloaded function) [Ex: findNearestHeading(270, 0) returns -90]
      -NormalizeAngle             Used in angle calculations to take the angle back to 0-360 value (can't use modulus on negative numbers without adverse effects)
      -CalculateRotation          helperfunction for findNearestHeading
    */
    void moveCradleSubroutine();
    void moveTurret(int16_t heading, TurretUnits units, float power = QB_HOME_PCT, bool relativeToRobot = true, bool ramp = false); 
    void updateTurretMotionStatus();
    int16_t getCurrentHeading();
    int16_t findNearestHeading(int16_t targetHeading, int16_t currentHeading);
    int16_t findNearestHeading(int16_t targetHeading);
    int NormalizeAngle(int angle);
    int CalculateRotation(float currentAngle, float targetAngle);

  public:
    QuarterbackTurret(
      uint8_t flywheelLeftPin,    // M1
      uint8_t flywheelRightPin,   // M2
      uint8_t cradlePin,          // M3
      uint8_t turretPin,          // M4
      uint8_t assemblyStepPin,    // S1
      uint8_t assemblyDirPin,     // S2 
      uint8_t magnetometerSdaPin, // S3
      uint8_t magnetometerSclPin, // S4
      uint8_t turretEncoderPinA,  // E1A
      uint8_t turretEncoderPinB,  // E1B
      uint8_t turretLaserPin      // E2A
    );

    bool magnetometerCalibrated = false;

    /*PUBLIC MSC AND SETUP FUNCTION DECLARATIONS
      -action                         robot sublass must override action
      -zeroTurret                     Runs through the routine to zero the turret
      -reset                          zero turret, aim down (straight), and set flywheels to slow intake
      -setEnabled                     toggles turret and flywheel movement
      -emergencyStop                  If the E-Stop button on controller is pressed this should override every other motor function and force them to stop
      -testForDisableOrStop           Tests what state the motors should be in
      -printDebug                     Consolidating printouts for the different functions into one locations for debugging purposes
    */
    void action() override;
    void zeroTurret();
    void reset();
    void setEnabled(bool enabled);
    void emergencyStop();
    bool testForDisableOrStop();
    void printDebug();

   /*QB DIRECT CONTROL FUNCTION DELCARATIONS
      -setTurretSpeed                 Moves the turret at a specified speed (open loop)
      -moveTurret                     Turn the turret to a specific heading (currently relative to robot) -> asynchronous
      -moveTurretAndWait              Moves the turret and waits until the heading is reached -> synchronous
      -aimAssembly                    Moves the falcon pylon assembly up and down between two states
      -moveCradle                     Moves teh cradel between two states throw and hold
      -setFlywheelSpeed               Sets the speed of the flywheels using stick inputs
      -setFlywheelSpeedStage          Set the speed as one of the defined enums
      -adjustFlywheelSpeedStage       Move to the next level up or down in the list of speeds
   */
    void setTurretSpeed(float absoluteSpeed, bool overrideEncoderTare = false); 
    void moveTurret(int16_t heading, bool relativeToRobot = true, bool ramp = true); 
    void moveTurret(int16_t heading, float power = QB_HOME_PCT, bool relativeToRobot = true, bool ramp = true);
    void moveTurretAndWait(int16_t heading, float power = QB_HOME_PCT, bool relativeToRobot = true, bool ramp = true);
    void aimAssembly(AssemblyAngle angle); 
    void moveCradle(CradleState state, bool force = false); 
    void setFlywheelSpeed(float absoluteSpeed); 
    void setFlywheelSpeedStage(FlywheelSpeed stage); 
    void adjustFlywheelSpeedStage(SpeedStatus speed);

    /*QB AUTOMATIC TARGETING
      -switchMode                 Used to switch between manual and automatic mode (overloaded function)
      -switchTarget               Switch between reciever1 or reciever2

    */
    void switchMode(TurretMode mode); 
    void switchMode(); 
    void switchTarget(TargetReceiver target); 

    /*QB MACROS
      -loadFromCenter             Prepared the QB to accept the ball from the center
      -handoff                    Hands the ball to the runningback
      -testRoutine                used for testing different functions
    */
    void loadFromCenter();
    void handoff();
    void testRoutine();

   

    /*ENCODER VARIABLES
      -currentTurretEncoderCount
      -turretEncoderStateB                  The A channel will be 1 when the interrupt triggers
    */
    static int32_t currentTurretEncoderCount;
    static uint8_t turretEncoderStateB;

    /*ENCODER FUNCTIONS
      -turretEncoderISR
      -turretDirectionChanged               IF the direction changes then some things need to happen differently because of the incredible amount of backlash
    */
    static void turretEncoderISR();
    void turretDirectionChanged();

    /* UART Communication Variables and Functions
    */
    int motor1Value = 0;
    int motor2Value = 0;
    void updateReadMotorValues();
};

#endif // QUARTERBACK_TURRET_H
