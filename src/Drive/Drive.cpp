#include <Arduino.h>
#include "Drive/Drive.h"
#include "Robot/MotorControl.h"

    Adafruit_MPU6050* mpu = new Adafruit_MPU6050();  
    sensors_event_t a, g, temp;

/**
 * @brief Drive Class, base class for specialized drive classes, this configuration is intended for the standard linemen.
 * this class takes the stick input, scales the turning value for each motor and ramps that value over time,
 * then sets the ramped value to the motors
 * @authors Rhys Davies (@rdavies02), Max Phillips (@RyzenFromFire)
 *
 * @class
 *    2 motor configuration shown below
 *
 *               ^
 *               | Fwd
 *       _________________ 
 *      |        _        |
 *      |       |O|       |       O: represents the Omniwheel, a wheel that can turn on 2 axis
 *      |       |_|       |       L: represents the left Wheel, powered by the left motor via a chain
 *      |  _           _  |            - the left motor would turn ccw to move the bot forward
 *      | |L|         |R| |       R: represents the right Wheel, powered by the right motor via a chain
 *      | |_|         |_| |            - the right motor would turn cw to move the bot forward
 *      |_________________|
 *
 * @todo
 *  - add a turning radius parameter, needed for the kicker
 *  - add mechanium driving code, for the new center, needed next semester (Spring 2023)
 *
 * Default configuration:
 * @param leftmotorpin the arduino pin needed for the left motor, needed for servo
 * @param rightmotorpin the arduino pin needed for the right motor, needed for servo
*/

Drive::Drive() {
  Drive(lineman, big_ampflow, {1, 9, 6, 36});
}

Drive::Drive(BotType botType, MotorType motorType) {
  Drive(botType, motorType, {1, 9, 6, 36});
}

Drive::Drive(BotType botType, MotorType motorType, drive_param_t driveParams, bool hasEncoders, int turnFunction, bool hasGyro) {
  this->botType = botType;
  this->motorType = motorType;
  this->hasEncoders = hasEncoders;
  this->hasGyro = false;
  this->gearRatio = driveParams.gear_ratio;
  this->wheelBase = driveParams.wheel_base;
  this->R_Min = driveParams.r_min;
  this->R_Max = driveParams.r_max;

  if (botType == quarterback_old) {
    this->BIG_BOOST_PCT = 0.8; 
    this->BIG_NORMAL_PCT = 0.4; 
    this->BIG_SLOW_PCT = 0.3;
  } else if (botType == center) {
    this->BIG_BOOST_PCT = 0.5;  
    this->BIG_NORMAL_PCT = 0.4; 
    this->BIG_SLOW_PCT = 0.25;
  } else {
    this->BIG_BOOST_PCT = 0.6;  
    this->BIG_NORMAL_PCT = 0.4; 
    this->BIG_SLOW_PCT = 0.1;
  }

  // initialize arrays
  for (int i = 0; i < NUM_MOTORS; i++) {
    requestedMotorPower[i] = 0.0f;
    lastRampPower[i] = 0.0f;
    turnMotorValues[i] = 0.0f;
  }

  if (botType != mecanum_center) {
    // initialize parameters for turning model
    omega = 0;
    omega_L = 0, omega_R = 0;
    R = 0.0f;
    // R_Max = 24.0f;
    // R_Max = 36.0f;
    // R_Min = wheelBase/2 + 4;
    min_RPM = 200;
    // max_RPM = M1.Percent2RPM(1);
    // max_RPM = M1.max_rpm;

    // initialize turn sensitivity variables
    enableTurnSensitivity = 0; //turnFunction; // 0 for linear, 1 for Rhys's function, 2 for cubic
    turnSensitivityScalar = 0.49; // Range: (0, 0.5) really [0.01, 0.49]
    domainAdjustment = 1/log((1-(turnSensitivityScalar + 0.5))/(turnSensitivityScalar + 0.5));
    power = 0;
    lastTime = 0;
  } 

  // Gyro
  if (hasGyro){
    CL_enable = true;

    switch (botType) {
        case BotType::lineman: { k_p = 1500; break; }
        case BotType::receiver: { k_p = 1500; break; }
        case BotType::runningback: { k_p = 1500; break; }
        case BotType::quarterback: { k_p = 1500; break; }
    }

    k_i = 0;

    integral_sum = 0;
    prev_current_error = 0;
    prev_integral_time = 0;

    mpu->begin(0x68);
    mpu->setGyroRange(MPU6050_RANGE_250_DEG);  // 250, 500, 1000, 2000
    mpu->setFilterBandwidth(MPU6050_BAND_260_HZ);  // 260, 184, 94, 44, 21, 10, 5
  }
}

void Drive::setupMotors(uint8_t lpin, uint8_t rpin) {
    //this->motorPins[0] = lpin, this->motorPins[1] = rpin;
    // this->M1 = new MotorControl(motorType, false, this->gearRatio);
    // this->M2 = new MotorControl(motorType, false, this->gearRatio);

    // M1->setup(lpin), M2->setup(rpin);
    M1.setup(lpin, this->motorType, this->gearRatio);
    M2.setup(rpin, this->motorType, this->gearRatio);
}


void Drive::setMotorType(MotorType motorType) {
    this->motorType = motorType;
}


/**
 * setStickPwr takes the stick values passed in and normalizes them to values between -1 and 1
 * and sets this value to the private variables stickFwdRev and stickTurn respectively
 * @author Rhys Davies
 * Created: 9-12-2022
 *
 * @param leftY the forward backward value from the left stick an unsigned 8-bit float (0 to 255)
 * @param rightX the left right value from the right stick an unsigned 8-bit float (0 to 255)
*/
void Drive::setStickPwr(int8_t leftY, int8_t rightX) {
    // left stick all the way forward is 0, backward is 255
    // +: forward, -: backward. needs to be negated so that forward is forward and v.v.; subtracting 1 bumps into correct range
    stickForwardRev = (leftY / 127.5f);
    stickTurn = (rightX / 127.5f);

    // stick deadzones
    // set to zero (no input) if within the set deadzone
    // subtacting STICK_DEADZONE and deviding by 1-STICK_DEADZONE normalize the inputs to use the full 0-1 range
    if (fabs(stickForwardRev) < STICK_DEADZONE)
      stickForwardRev = 0;
    else if (stickForwardRev > 0)
      stickForwardRev = (stickForwardRev - STICK_DEADZONE) / (1 - STICK_DEADZONE);
    else if (stickForwardRev < 0)
      stickForwardRev = (stickForwardRev + STICK_DEADZONE) / (1 - STICK_DEADZONE);
    
    if (fabs(stickTurn) < STICK_DEADZONE)
      stickTurn = 0;
    else if (stickTurn > 0)
      stickTurn = (stickTurn - STICK_DEADZONE) / (1 - STICK_DEADZONE);
    else if (stickTurn < 0)
      stickTurn = (stickTurn + STICK_DEADZONE) / (1 - STICK_DEADZONE);

}

float Drive::getForwardPower() {
    return stickForwardRev;
}

float Drive::getTurnPower() {
    return stickTurn;
}

/**
 * @brief setBSN sets the internal variable to the requested percent power, this is what the motor power gets multiplied by,
 * this is where the boost, slow and normal scalars get passed in
 * @author Rhys Davies
 * Created: 9-12-2022
 *
 * @param bsn input speed choice Drive::Boost, Drive::Slow, Drive::Normal
*/
void Drive::setBSN(Speed bsn) {
    // set the scalar to zero if the requested value is greater than 1, this is not entirely necessary, but is a safety
    switch (bsn) {
        case BOOST: {
            switch (motorType) {
                case MotorType::big_ampflow: { BSNscalar = BIG_BOOST_PCT; break; }
                case MotorType::small_ampflow: { BSNscalar = SMALL_BOOST_PCT; break; }
                case MotorType::mecanum: { BSNscalar = MECANUM_BOOST_PCT; break; }
                case MotorType::falcon: { BSNscalar = FALCON_BOOST_PCT; break; }
                case MotorType::small_12v: { BSNscalar = SMALL_12V_BOOST_PCT; break; }
            }
            break;
        }
        case NORMAL: {
            switch (motorType) {
                case MotorType::big_ampflow: { BSNscalar = BIG_NORMAL_PCT; break; }
                case MotorType::small_ampflow: { BSNscalar = SMALL_NORMAL_PCT; break; }
                case MotorType::mecanum: { BSNscalar = MECANUM_NORMAL_PCT; break; }
                case MotorType::falcon: { BSNscalar = FALCON_NORMAL_PCT; break; }
                case MotorType::small_12v: { BSNscalar = SMALL_12V_NORMAL_PCT; break; }
            }
            break;
        }
        case SLOW: {
            switch (motorType) {
                case MotorType::big_ampflow: { BSNscalar = BIG_SLOW_PCT; break; }
                case MotorType::small_ampflow: { BSNscalar = SMALL_SLOW_PCT; break; }
                case MotorType::mecanum: { BSNscalar = MECANUM_SLOW_PCT; break; }
                case MotorType::falcon: { BSNscalar = FALCON_SLOW_PCT; break; }
                case MotorType::small_12v: { BSNscalar = SMALL_12V_SLOW_PCT; break; }
            }
            break;
        }
        case BRAKE: {
            BSNscalar = BRAKE_BUTTON_PCT;
            break;
        }
    }
}

void Drive::setBSNValue(float bsn_pct) {
    BSNscalar = bsn_pct;
}

float Drive::getBSN() {
    return this->BSNscalar;
}

/**
 * generateTurnScalar takes the input stick power and scales the max turning power allowed with the forward power input
 * @authors Grant Brautigam, Rhys Davies, Max Phillips
 * Created: 9-12-2022
*/
void Drive::generateMotionValues(float tankModePct) {
    if (fabs(stickForwardRev) < STICK_DEADZONE) { // fwd stick is zero
        drivingStraight = false;
        if (fabs(stickTurn) < STICK_DEADZONE) { // turn stick is zero
            requestedMotorPower[0] = 0, requestedMotorPower[1] = 0; // not moving, set motors to zero
        } else if (stickTurn > STICK_DEADZONE) { // turning right, but not moving forward much so use tank mode
            requestedMotorPower[0] = BSNscalar * abs(stickTurn)  * tankModePct;
            requestedMotorPower[1] = -BSNscalar * abs(stickTurn) * tankModePct;
        } else if (stickTurn < -STICK_DEADZONE) { // turning left, but not moving forward much so use tank mode
            requestedMotorPower[0] = -BSNscalar * abs(stickTurn) * tankModePct;
            requestedMotorPower[1] = BSNscalar * abs(stickTurn)  * tankModePct;
        } // no general else since encountered infinite loop
    } else { // fwd stick is not zero
        if (fabs(stickTurn) < STICK_DEADZONE) { // turn stick is zero
            // just move forward directly
            drivingStraight = true;
            requestedMotorPower[0] = BSNscalar * stickForwardRev;
            requestedMotorPower[1] = BSNscalar * stickForwardRev;
        } else { // moving forward and turning
            /*
            if the sticks are not in any of the edge cases tested for above (when both sticks are not 0),
            a value must be calculated to determine how to scale the motor that is doing the turning.
            i.e.: if the user moves the left stick all the way forward (stickFwdRev = 1), and they are attempting
            to turn right. The left motor should get set to 1 and the right motor should get set to
            some value less than 1, this value is determined by the function calcTurningMotorValue
            */
            drivingStraight = false;
            if (stickTurn > STICK_DEADZONE) { // turn Right
                // switch(abs((BSNscalar * stickForwardRev)) > abs(lastRampPower[0])) {
                //     case true: calcTurning(stickTurn, abs(lastRampPower[0])); break;
                //     case false: calcTurning(stickTurn, abs(BSNscalar * stickForwardRev)); break;
                // }
                calcTurning(abs(stickTurn), abs(BSNscalar * stickForwardRev));

                requestedMotorPower[0] = copysign(turnMotorValues[0], stickForwardRev);
                requestedMotorPower[1] = copysign(turnMotorValues[1], stickForwardRev);
            } else if (stickTurn < -STICK_DEADZONE) { // turn Left
                // switch(abs((BSNscalar * stickForwardRev)) > abs(lastRampPower[1])) {
                //     case true: calcTurning(stickTurn, abs(lastRampPower[1])); break;
                //     case false: calcTurning(stickTurn, abs(BSNscalar * stickForwardRev)); break;
                // }

                calcTurning(abs(stickTurn), abs(BSNscalar * stickForwardRev));
                
                requestedMotorPower[0] = copysign(turnMotorValues[1], stickForwardRev);
                requestedMotorPower[1] = copysign(turnMotorValues[0], stickForwardRev);
            }
        }
    }
}


/**
 * @brief calcTurning generates power values for each motor to achieve a given turn
 * @authors Grant Brautigam, Rhys Davies
 * Created: 9-12-2022
 * updated: 10-6-2023
 * Mathematical model:
 * Whitepaper: https://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf
 * 
 * Add link to Desmos in future
 * 
 * Differential drive turning model
 * 
 * omega_R = (omega/R)*(R + l/2)
 * omega_L = (omega/R)*(R - l/2)
 *
 * omega_R: right wheel angular velocity
 * omega_R: left wheel angular velocity
 * omega: the rate of rotation around the ICC (Instantaneous Center of Curvature)
 * l: distance between each wheel (wheelbase) 
 * R: distance the ICC from the center of the wheelbase (l/2)
 * 
 * @param stickTrn the absoulte value of the current turning stick input
 * @param fwdLinPwr the non-turning motor value from the previous loop, which was actually sent to the motor
 */
void Drive::calcTurning(float stickTrn, float fwdLinPwr) {
    //R_Min = R_Min + abs(stickForwardRev)*(R_High_Min - R_Min); // start of turn scaling 
    if (enableTurnSensitivity == 0) // linear
        scaledSensitiveTurn = stickTrn;
    else if(enableTurnSensitivity == 1) // rhys's function
        scaledSensitiveTurn = log((1-(turnSensitivityScalar * stickTrn + 0.5))/(turnSensitivityScalar * stickTrn + 0.5)) * domainAdjustment;
    else // cubic
        scaledSensitiveTurn = pow(stickTrn, 3);
        
    // Calculate the R value from the stick turn input
    R = (1-scaledSensitiveTurn)*(R_Max-R_Min) + R_Min;
    
    // calculate the requested angular velocity for the robot 
    omega = abs(M1.Percent2RPM(fwdLinPwr));

    // calculate the rpm for the left wheel
    omega_L = (omega/R)*(R+(wheelBase/2));
    // calculate the rpm for the right wheel
    omega_R = (omega/R)*(R-(wheelBase/2));

    // ensure the left wheel RPM doesnt go below the min or above the max RPM
    omega_L = constrain(omega_L, min_RPM, M1.max_rpm);
    // ensure the left wheel RPM doesnt go below the min or above the max RPM
    omega_R = constrain(omega_R, min_RPM, M1.max_rpm);

    turnMotorValues[0] = M1.RPM2Percent(omega_L);
    turnMotorValues[1] = M2.RPM2Percent(omega_R);
}

void Drive::emergencyStop() {
    M1.stop(); 
    M2.stop();
}

void Drive::printSetup() {
    Serial.print(F("\nDrive::printSetup():"));
    Serial.print(F("\nMotorType: "));
    Serial.print(getMotorTypeString(this->motorType));
    Serial.print(F("\nGearRatio: "));
    Serial.print(this->gearRatio);
    Serial.print(F("\nR_Min: "));
    Serial.print(this->R_Min);
    Serial.print(F("\nR_Max: "));
    Serial.print(this->R_Max);
    Serial.print(F("\nMin RPM: "));
    Serial.print(this->min_RPM);
    Serial.print(F("\nMAX RPM: "));
    Serial.print(M1.max_rpm);
    Serial.print(F("\nTurnSensitivityMode: "));
    Serial.print(enableTurnSensitivity);
    Serial.print(F("\nEncoders: "));
    Serial.print(F("\nHas Encoders? "));
    Serial.print(this->hasEncoders ? F("True") : F("False"));

    Serial.print(F("\n"));
}

/**
 * prints the internal variables to the serial monitor in a clean format,
 * this function exists out of pure laziness to not have to comment out all the print statments
 * @author
 * Updated:
*/
void Drive::printDebugInfo() {
    Serial.print(F("L_Hat_Y: "));
    Serial.print(stickForwardRev);
    Serial.print(F("  R_HAT_X: "));
    Serial.print(stickTurn);

    // Serial.print(F("  |  Turn: "));
    // Serial.print(lastTurnPwr);

    // Serial.print(F("  |  Left ReqPwr: "));
    // Serial.print(requestedMotorPower[0]);
    // Serial.print(F("  Right ReqPwr: "));
    // Serial.print(requestedMotorPower[1]);

    Serial.print(F("  |  Omega: "));
    Serial.print(omega);

    Serial.print(F("  omega_L: "));
    Serial.print(omega_L);
    Serial.print(F("  omega_R: "));
    Serial.print(omega_R);

    // Serial.print(F("  lastRampTime "));
    // Serial.print(lastRampTime[0]);
    // Serial.print(F("  requestedPower "));
    // Serial.print(requestedPower);
    // Serial.print(F("  current "));
    // Serial.print(currentRampPower[0]);
    // Serial.print(F("  requestedPower - currentRampPower "));
    // Serial.println(requestedPower - currentRampPower[mtr], 10);

    Serial.print(F("  Left Motor: "));
    Serial.print(requestedMotorPower[0]);
    Serial.print(F("  Right: "));
    Serial.print(requestedMotorPower[1]);

    //Serial.print(F("  scaledSensitiveTurn: "));
    //Serial.print(scaledSensitiveTurn);

    Serial.print(F("\n"));
}

/**
 * @brief Prints variables to the serial monitor in a csv format
 * This function is important for data acquisition
 * The options below are configurable, change them as you need
 * Remember to adhere to printing guidelines under PR-Docs
 * @author Corbin Hibler
 * Updated: 2023-10-30
*/
void Drive::printCsvInfo() {
    Serial.print(F("header1,")); // name of value to be used as header
    Serial.print(1);             // variable you want to track
    Serial.print(F(",header2,")); 
    Serial.print(2);
    Serial.print(F(",header3,"));
    Serial.print(3);
    Serial.print(F(",header4,"));
    Serial.print(4);
    Serial.print(F(",header5,"));
    Serial.println(5); // last line is -ALWAYS- println or else the python script will break
}

void Drive::setCurrentAngleSpeed(float speed) {
  currentAngleSpeed = speed;
}

/**
 * @brief NEEDS SPELLCHECK integrate uses trapizodial intgration to calculate the running integral sum for the PI controller
 * 
*/
int Drive::integrate(int current_error) {
  integral_sum = integral_sum + (current_error + prev_current_error); //*(millis()-prev_integral_time)/100;
  prev_integral_time = millis();
  prev_current_error = current_error;
  
  return integral_sum; 
}

/**
 * @brief NEEDS SPELLCHECK integrateReset resets the varibles in integral
 * 
*/
void Drive::integrateReset() {
  integral_sum = 0;
  prev_current_error = 0;
  prev_integral_time = millis();
}

/**
 * @brief NEEDS SPELLCHECK PILoop is the closed loop controller. this is the main function for CL  
 * @author Grant Brautigam
 * Updated 11-19-2023
 * 
*/
int Drive::PILoop() {  
  if (abs(currentAngleSpeed) >= ERROR_THRESHOLD) { // the motor wants to stop, skip and reset the PI loop  
    motorDiffCorrection = k_p*currentAngleSpeed + k_i*integrate(currentAngleSpeed);
  } else {
    motorDiffCorrection = 0;
    integrateReset();
  }

  return motorDiffCorrection; 
}

/**
 * 
 * @brief updates the motors after calling all the functions to generate
 * turning and scaling motor values, the intention of this is so the
 * programmer doesnt have to call all the functions, this just handles it,
 * reducing clutter in the main file.
 * DO NOT CALL THIS FUNCTION UNTIL setStickPwr and setBSN have been called before update
 * @author Rhys Davies
 * Created: 9-12-2022
 * Updated: 10-11-2020
*/
void Drive::update() {
    // Generate turning motion
    generateMotionValues();
    //delay(100);
    if (CL_enable) {
        mpu.getEvent(&a, &g, &temp);
        setCurrentAngleSpeed(g.gyro.z - 0.03);
        motorDiff = PILoop()*.5;
        Serial.print(motorDiff);
        Serial.print("  ");
        Serial.print("Rotation Z, ");
        Serial.print(g.gyro.z - 0.03);
        Serial.println("");
    } else {
        motorDiff = 0;
    }

    if (drivingStraight){
        M1.setTargetSpeed(M1.Percent2RPM(requestedMotorPower[0]) + motorDiff); // results in 800ish rpm from encoder
        M2.setTargetSpeed(-M2.Percent2RPM(requestedMotorPower[1]) - motorDiff); // results in 800ish rpm from encoder
    } else {
        M1.setTargetSpeed(M1.Percent2RPM(requestedMotorPower[0])); // results in 800ish rpm from encoder
        M2.setTargetSpeed(-M2.Percent2RPM(requestedMotorPower[1])); // results in 800ish rpm from encoder
    }
    
    //printDebugInfo();

}

void Drive::setCurrentAngelSpeed(float speed) {
  currentAngleSpeed = speed;
}

/**
 * @brief NEEDS SPELLCHECK integrate uses trapizodial intgration to calculate the running integral sum for the PI controller
 * 
*/
int Drive::integrate(int current_error) {

  integral_sum = integral_sum + (current_error + prev_current_error); //*(millis()-prev_integral_time)/100;
  prev_integral_time = millis();
  prev_current_error = current_error;
  
  return integral_sum; 
}

/**
 * @brief NEEDS SPELLCHECK integrateReset resets the varibles in integral
 * 
*/
void Drive::integrateReset() {
  integral_sum = 0;
  prev_current_error = 0;
  prev_integral_time = millis();
}

/**
 * @brief NEEDS SPELLCHECK PILoop is the closed loop controller. this is the main function for CL  
 * @author Grant Brautigam
 * Updated 11-19-2023
 * 
*/
int Drive::PILoop() {  

  if (abs(currentAngleSpeed) >= ERROR_THRESHOLD) {// the motor wants to stop, skip and reset the PI loop  
    
    motorDiffCorrection = k_p*currentAngleSpeed + k_i*integrate(currentAngleSpeed);
    
  } else {

    motorDiffCorrection = 0;
    integrateReset();

  }

  return motorDiffCorrection; 

}

void Drive::update2(int speedL, int speedR) {

    generateMotionValues();

    M1.setCurrentSpeed(speedL);
    M2.setCurrentSpeed(speedR);

    if (CL_enable) {
        motorDiff = PILoop()*.5;
        Serial.print(motorDiff);
        Serial.print("  ");
    } else {
        motorDiff = 0;
    }
    //M1.write(-.3);
    //M1.setTargetSpeed(-900);
    if (drivingStraight){
        M1.setTargetSpeed(M1.Percent2RPM(requestedMotorPower[0]) + motorDiff); // results in 800ish rpm from encoder
        M2.setTargetSpeed(M2.Percent2RPM(requestedMotorPower[1]) - motorDiff); // results in 800ish rpm from encoder
    } else {
        M1.setTargetSpeed(M1.Percent2RPM(requestedMotorPower[0])); // results in 800ish rpm from encoder
        M2.setTargetSpeed(M2.Percent2RPM(requestedMotorPower[1])); // results in 800ish rpm from encoder
    }

    if ((millis() - lastTime) >= 100) {
        power = power - 0.05;
        //power = M1.RPM2Percent(speed);
        lastTime = millis();
    }
    //2 target and 2 actual and millis
    Serial.print("millis,");
    Serial.print(millis());
    Serial.print(",left encoder target speed,");
    Serial.print(M1.Percent2RPM(requestedMotorPower[0]) + motorDiff);
    Serial.print(",left encoder actual speed,");
    Serial.print(speedL);
    Serial.print(",right encoder target speed,");
    Serial.print(M2.Percent2RPM(requestedMotorPower[1]) - motorDiff);
    Serial.print(",right encoder actual speed,");
    Serial.print(speedR);
    Serial.print(",BSN");
    Serial.println(BSNscalar);

    
}