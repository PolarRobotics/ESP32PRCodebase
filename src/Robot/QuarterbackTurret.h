#pragma once

#ifndef QUARTERBACK_TURRET_H
#define QUARTERBACK_TURRET_H

#include <Robot/Robot.h>
#include <Robot/MotorControl.h>
#include <ps5Controller.h> // ESP PS5 library, access using global instance `ps5`
#include <Utilities/Debouncer.h>
#include <Adafruit_LIS3MDL.h>
#include <ADXL335.h>

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

#define QB_TURRET_NUM_SPEEDS 7

enum FlywheelSpeed {
  slow_inwards, stopped, slow_outwards, lvl1_outwards, lvl2_outwards, lvl3_outwards, maximum
};

const float flywheelSpeeds[QB_TURRET_NUM_SPEEDS] = {-0.1, 0, 0.1, 0.3, 0.5, 0.7, 1.0};

#define QB_BASE_DEBOUNCE_DELAY 50L // 50 millseconds for button debouncing by default

#define QB_CRADLE_TRAVEL_DELAY 750L // ~0.75 seconds to fully extend or compress linear actuator

#define QB_CIRCLE_HOLD_DELAY 750L
#define QB_TRIANGLE_HOLD_DELAY 200L
#define QB_CROSS_HOLD_DELAY 200L

#define QB_TURRET_INTERPOLATION_DELAY 5L
#define QB_TURRET_THRESHOLD 35
#define QB_TURRET_STICK_SCALE_FACTOR 0.25

#define QB_MIN_PWM_VALUE 0.08
#define QB_HOME_PCT 0.125
#define QB_HANDOFF  0.3
#define QB_HOME_MAG 0.1

//* Question: Why are there so many #defines commented out?
//* Answer: Documentation purposes.

//* Question: Why not just use the #defines as they are written to generate the values?
//* Answer: I don't trust the preprocessor. -MP

#define QB_COUNTS_PER_ENCODER_REV 1000
// 27:1 from falcon to output (1 rev of falcon is 1/27th of turret)
// 5:1 from falcon to encoder gear (12t on falcon, 60t on encoder)
// big gear on encoder needs to spin 5.4 (27/5) times for the turret to make one revolution
// 1 revolution on big gear is the same as 1 revolution on the encoder
// so 5:27 is encoder:turret, and:
// 5/27ths of a revolution of the turret for 1 revolution of the encoder

// #define QB_FALCON_TO_TURRET_RATIO 27 / 1
// #define QB_ENCODER_TO_FALCON_RATIO 5 / 1
// #define QB_ENCODER_TO_TURRET_RATIO /* = */ QB_FALCON_TO_TURRET_RATIO / QB_ENCODER_TO_FALCON_RATIO // (27 / 5) = 5.4
// #define QB_COUNTS_PER_TURRET_REV /* = */ QB_ENCODER_TO_TURRET_RATIO * QB_COUNTS_PER_ENCODER_REV // 5400

#define QB_COUNTS_PER_TURRET_REV /* = */ 5400

// #define QB_COUNTS_PER_TURRET_DEGREE /* = */ QB_COUNTS_PER_TURRET_REV / 360
#define QB_COUNTS_PER_TURRET_DEGREE 15

// there is a large amount of mechanical slop when changing turret directions due to the large gear reduction
// it takes about 10 gear teeth's worth of movement of the encoder gear before the turret actually begins moving again
// for 1000 counts per encoder rev => 5400 counts per turret rev, 10 / 100 gear teeth * 5400 counts / turret = 540 counts

// #define QB_TURRET_SLOP_GEAR_TEETH /* = */ 10 // slop is about 10 gear teeth
// #define QB_TURRET_SLOP_PCT        /* = */ QB_DIRECTION_CHANGE_SLOP_GEAR_TEETH / 100
// #define QB_TURRET_SLOP_COUNTS     /* = */ QB_DIRECTION_CHANGE_SLOP_PCT * QB_COUNTS_PER_TURRET_REV
#define QB_TURRET_SLOP_COUNTS 540

#define QB_TURRET_STOP_LOOP_DELAY_MS 10
#define QB_TURRET_STOP_THRESHOLD_MS 500 // must be a multiple of QB_TURRET_STOP_LOOP_DELAY_MS

#define QB_TURRET_HOME_STOP_FACTOR 0.5 // correction constant for homing, multiplied into stop counts in final homing

// higher means the turret will be less sensitive during manual control.
// this defines the limit of the counter that basically "divides the clock"
// of the loop so that the turret only updates every X iterations
#define QB_TURRET_MANUAL_CONTROL_FACTOR 4 

#define QB_TURRET_PID_THRESHOLD 3
#define QB_TURRET_PID_MIN_DELTA_T 5
#define QB_TURRET_PID_MAX_DELTA_T 25
#define QB_TURRET_PID_BAD_DELTA_T 250

#define QB_NORTH_OFFSET 0

//* Enable or Disable Auto Mode for testing
#define QB_AUTO_ENABLED false


// Accelerometer Constants
#define ACCEL_DEADZONE 0.15
#define ACCEL_AVG_SIZE 25
#define ACCEL_CALIB_AVG_SIZE 100

/**
 * @brief Quarterback Turret Subclass Header
 * @authors Maxwell Phillips
 */
class QuarterbackTurret : public Robot {
  private: 

    //* motor instances
    MotorControl cradleActuator;
    MotorControl turretMotor;
    MotorControl flywheelLeftMotor;
    MotorControl flywheelRightMotor;
    
    //* pins that must be persistent
    static uint8_t turretEncoderPinA;
    static uint8_t turretEncoderPinB;
    uint8_t turretLaserPin;

    //* joystick inputs
    float stickTurret;   // used to normalize stick input from [0, 255] to [-1.0, 1.0]
    float stickFlywheel; // same as above

    // mode
    TurretMode mode; // default manual
    TargetReceiver target; // default receiver_1

    //* state
    // default false, set to true upon homing/reset, but toggleable via function
    bool enabled; 

    // default false, set to true upon homing/reset. once true, stays true
    // determines whether or not to initiate entire startup/reset routine or just zero the turret
    bool initialized; 

    // default false, set to true while a macro (loadFromCenter or handoff) is running
    bool runningMacro;

    //* assembly
    AssemblyAngle currentAssemblyAngle; // initial state unknown, will reset to straight
    AssemblyAngle targetAssemblyAngle; // default straight
    bool assemblyMoving; // default false

    //* cradle
    CradleState currentCradleState; // initial state unknown, will reset to back
    CradleState targetCradleState; // default back
    bool cradleMoving; // default false
    uint32_t cradleStartTime;


    //* flywheel
    FlywheelSpeed currentFlywheelStage; // default stopped
    FlywheelSpeed targetFlywheelStage;  // default stopped

    float currentFlywheelSpeed; // default 0
    float targetFlywheelSpeed;  // default 0

    bool flywheelManualOverride; // default false, true when stick controlling flywheel

    //* turret
    float currentTurretSpeed; // default 0
    float targetTurretSpeed;  // default 0

    // robot-relative headings
    int16_t currentRelativeHeading; // default undefined
    int16_t targetRelativeHeading;  // default 0

    int32_t currentRelativeTurretCount; // default undefined
    int32_t targetTurretEncoderCount; // default 0
    int32_t errorEncoderCount; // error, i.e. |current - target|
    int32_t slopError; // error due to mechanical slop, automatically determined by robot during reset
    int32_t stopError; // number of encoder counts that are needed for the motor to stop

    bool turretMoving;
    uint8_t manualHeadingIncrementCount; // default 0

    // world-relative headings
    int16_t currentAbsoluteHeading; // default 0
    int16_t targetAbsoluteHeading;  // default 0

    uint8_t turretLaserState;

    // debouncers
    Debouncer* dbOptions;
    Debouncer* dbSquare;
    Debouncer* dbDpadUp;
    Debouncer* dbDpadDown;

    // debouncers for delay
    Debouncer* dbCircle;
    Debouncer* dbTriangle;
    Debouncer* dbCross;
    Debouncer* dbTurretInterpolator;

    /******* ACCELEROMETER *******/
    ADXL335 accelerometer;
    float ax,ay,az;

    /* Calibration Variables 
        - calibrationXSum, calibrationYSum:       Adds up over multiple loops to get an average for when robot is sitting still
        - calibrationXValue, calibrationYValue:   The average value used to get acceleration values during main loop
    */
    float calibrationXValue = 0;
    float calibrationYValue = 0;
    float calibrationXSum = 0;
    float calibrationYSum = 0;

    /* Accelerometer Calculation Variables
        - movingAverageX, movingAverageY:                     Used to smooth out bumps in data while keeping accuracy
        - movingAverageXIndex, movingAverageYIndex:           The arrays act like queues so keeping track of what element we are on
        - calcAverage:                                        currentValue of the average
        - prevmovingAverageX, prevmovingAverageY:             Keeping track of the previous average values for use when deciding when to reset (used to remove error build up)
        - prevmovingAverageXIndex, prevmovingAverageYIndex:   The arrays act like queues so keeping track of what element we are on
        - prevMovingAvgMax, prevMovingAvgMin:                 Min and max values used to calculate the range found in the previous values (used to remove error build up)
        - maxXAccel, minXAccel, maxYAccel, minYAccel:         Used to find the range in values as the program runs
        - 
    */
    float movingAverageX[10];
    float movingAverageY[10];
    int movingAverageXIndex = 0;
    int movingAverageYIndex = 0;
    float calcAverage = 0.0;
    float prevmovingAverageX[100];
    float prevmovingAverageY[100];
    int prevmovingAverageXIndex = 0;
    int prevmovingAverageYIndex = 0;
    float prevMovingAvgMax = -100000;
    float prevMovingAvgMin = 100000;
    float maxXAccel = -1000000.0;
    float minXAccel = 1000000.0;
    float maxYAccel = -1000000.0;
    float minYAccel = 1000000.0;

    /* Final Accelerometer Values
        - currentAccelX, currentAccelY:         The current acceleration values in each Axis
        - accelXRunningSum, accelYRunningSum:   The current running sum in each axis, used to slow down QB turret
    */
    float currentAccelX;
    float currentAccelY = 0;
    float accelXRunningSum;
    float accelYRunningSum = 0;

    // put function declarations here:
    void accelerometerSetup();
    void calculateAcceleration();

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
        - headingError:   
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
    

    // private helper function to avoid code duplication between force and normal case
    void moveCradleSubroutine();

    // moves turret/turntable to specific heading. currently relative to robot, not field.
    //* private helper function
    void moveTurret(int16_t heading, TurretUnits units, float power = QB_HOME_PCT, bool relativeToRobot = true, bool ramp = false); 

    //* private helper function to allow managing turret movement asynchronously, and stop it when it reaches the target position
    void updateTurretMotionStatus();

    //* private helper function to calculate new value for currentRelativeHeading from currentTurretEncoderCount
    int16_t getCurrentHeading();

    //* private helper function to calculate the heading with the smallest absolute value difference from the current (or provided)
    // for example, if called with target = 270 degrees and current = 0, this function would return -90,
    // because it is closer to go counterclockwise 90 degrees than to go clockwise 270 degrees.
    // note that the current heading should be positive, but the negative sign on a target heading is used to determine direction.
    // overloaded to allow calling with currentRelativeHeading (cannot set a class member as a default argument)
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

    void action() override; //! robot subclass must override action

    //* base/internal functions
    // moves turret at specified speed (open loop)
    void setTurretSpeed(float absoluteSpeed, bool overrideEncoderTare = false); 

    // moves turret/turntable to specific heading. currently relative to robot, not field. (NOT BLOCKING)
    void moveTurret(int16_t heading, bool relativeToRobot = true, bool ramp = true); 
    void moveTurret(int16_t heading, float power = QB_HOME_PCT, bool relativeToRobot = true, bool ramp = true);

    // moves turret and loops/waits until heading is reached (BLOCKING/SYNCHRONOUS)
    void moveTurretAndWait(int16_t heading, float power = QB_HOME_PCT, bool relativeToRobot = true, bool ramp = true);
      
    // aims the assembly holding the flywheels and cradle (two states).
    void aimAssembly(AssemblyAngle angle); 

    // moves the linear actuator connected to the cradle to one of two states.
    // used to fire the ball in both manual and automatic modes
    void moveCradle(CradleState state, bool force = false); 
    
    // also accessible via manual stick override
    void setFlywheelSpeed(float absoluteSpeed); 

    // directly set flywheel speed to a stage
    void setFlywheelSpeedStage(FlywheelSpeed stage); 

    //* derived functions (general)
    void adjustFlywheelSpeedStage(SpeedStatus speed);

    //* derived functions (automatic targeting)
    // switch between manual and automatic targeting
    void switchMode(TurretMode mode); 

    // same as above but toggles automatically instead of explicitly
    void switchMode(); 

    // switch between targeted receivers. switches to automatic targeting if not already set.
    void switchTarget(TargetReceiver target); 

    //* derived functions (manual macros)
    void loadFromCenter(); // prepares qb to intake ball from center
    void handoff(); // hands off ball to runningback
    void testRoutine();

    //* setup and safety functions
    void zeroTurret(); // calibrates turret/moves turret to home/zero (cnc/3d printer style)
    void reset(); // zero turret, aim down (straight), and set flywheels to slow intake
    void setEnabled(bool enabled); // toggles turret and flywheel movement
    void emergencyStop();
    bool testForDisableOrStop();
    
    void printDebug();

    // * encoder
    static int32_t currentTurretEncoderCount;
    static uint8_t turretEncoderStateB; // A channel will be 1 when interrupt triggers
    static void turretEncoderISR();

    void turretDirectionChanged();
};

#endif // QUARTERBACK_TURRET_H
