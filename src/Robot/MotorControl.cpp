#include <Arduino.h>

#include "MotorControl.h"

// void ext_read_encoder0() {
//   GlobalClassPointer[0]->readEncoder();
// }

// void ext_read_encoder1() {
//   GlobalClassPointer[1]->readEncoder();
// }

/**
 * @brief 
 * A class similar to the servo class, implements a method of choosing timers and channels 
 * based on the number of motors attached, uses writeMicros, etc. Utilizes the LedC built-in functions
 * 
 * since the sabretooth is expecting a duty cycle difference (time the signal is high for) 
 * I wrote a function to convert the time on value to a duty cycle, this would need to be moved into the Control class,
 * 
 * Goals/ Design specs: 
 *  - be able to declare multiple motor objects, with pin definitions.
 *  - avoid two motors on one pin and timers overlapping
 *  - write a value to the motor, maybe -100 100, we can play around with this
 *  - needs to be usable in drive and special bot classes
 * 
 * Example uses:
 * Definition: the motor object type can simply be defined by naming the motor object using the Motor type:
 *      Motor ExampleMotorObj;
 *   
 * and the pin can be attached by calling attach():
 *      ExampleMotorObj.attach();
 * 
 * to write to the motor simply pass a float between -1 and 1 into write()
 *      ExampleMotorObj.write(power);
 * 
 * where -1, 0 and 1 correspond to the throttles in the reverse, stop and forward directions
 * 
 * !TODO:
 * allow for us values outside the 1000us to 2000us, allow user to input the min and max values and pwr2Duty will account for that range
 * 
 * @author Rhys Davies 
 */
MotorControl::MotorControl() {  
  requestedRPM = 0;
  lastRampTime = millis();

  CL_enable = true;
  k_p = 2;
  k_i = 0.15;
  //k_i = 0.05;

  integral_sum = 0;
  prev_current_error = 0;
  prev_integral_time = 0;

  // if (has_encoder) {
  //   this->encoderIndex = EncoderCount;
  //   GlobalClassPointer[EncoderCount++] = this;
  //   init_encoder();
  // }
}



/**
 * @brief setup the given pin to the next free channel, returns channel number or 255 if failure
 * @author Rhys Davies
 * Updated 2-26-2023
 * 
 * @param mot_pin the pin the motor is connected to
 * 
 * @return uint8_t the channel number the pin is attached to, 255 if failure
 */
uint8_t MotorControl::setup(int mot_pin, MotorType type, bool has_encoder, float gearRatio, int enc_a_chan_pin, int enc_b_chan_pin) {
  this->has_encoder = has_encoder;
  this->motor_type = type;
  this->gear_ratio = gearRatio;
  this->enc_a_pin = enc_a_chan_pin, this->enc_b_pin = enc_b_chan_pin;

  // Calculate the max rpm by multiplying the nominal motor RPM by the gear ratio
  this->max_rpm = int(MOTOR_MAX_RPM_ARR[static_cast<uint8_t>(this->motor_type)] * this->gear_ratio);

  // call the logic to attach the motor pin and setup, return 255 on an error
  return Motor.attach(mot_pin, MIN_PWM_US, MAX_PWM_US);
}

/**
 * @brief write, wrapper function for MotorInterface, to be removed in future
 * !TODO: remove in future
*/
void MotorControl::write(int rpm) {

  // curve fit for only big ampflow
  if (rpm < 0)
     pct = copysign(.0012f*pow(constrain(abs(rpm), -this->max_rpm, this->max_rpm), 0.7895f), rpm);

  pct = copysign(.0087f*pow(constrain(abs(rpm), -this->max_rpm, this->max_rpm), 0.5616f), rpm);

  Motor.write(pct);
}

/**
 * @brief NEEDS SPELLCHECK stop derectly writes o to the motor. It also resets the integral adder inside the PI loop. 
 * This is an esay way to rapidly stop motion on an indeviual motor
 * 
 */
void MotorControl::stop() {
    this->write(0);
    if (CL_enable)
      this->PILoop(0);
}

/**
 * @brief NEEDS SPELLCHECK setTargetSpeed is the "main loop" of motor control. It is the profered function to set the speed of a motor
 * it calls other inportent functions like ramp and PILoop
 * 
*/
void MotorControl::setTargetSpeed(int target_rpm) {
 //Serial.println("here");
  ramped_speed = ramp(target_rpm, 1200.0f); // first call ramp for traction control and to make sure the PI loop dose not use large accerations

  // if there are working encoders its safe to use the PL loop, 
  // if the encoder fails or is not present the PI loop MUST be bypased to aviod an out of control robot
  if (CL_enable) 
    set_speed = PILoop(ramped_speed);  
  else 
    set_speed = ramped_speed;

  this->write(set_speed); //convert speed to the coresponding motor power and write to the motor 

}

/**
 * @brief NEEDS SPELLCHECK getCurrentSpeed is the interface between the main CPU and the encoder modual 
 * 
*/
int MotorControl::getCurrentSpeed() {

  //! Quantum this is yours to write

  return current_speed; 
}

void MotorControl::setCurrentSpeed(int speed) {
 current_speed = speed;
}

/**
 * @brief NEEDS SPELLCHECK integrate uses trapizodial intgration to calculate the running integral sum for the PI controller
 * 
*/
int MotorControl::integrate(int current_error) {

  integral_sum = integral_sum + (current_error + prev_current_error); //*(millis()-prev_integral_time)/100;
  prev_integral_time = millis();
  prev_current_error = current_error;
  
  return integral_sum; 
}

/**
 * @brief NEEDS SPELLCHECK integrateReset resets the varibles in integral
 * 
*/
void MotorControl::integrateReset() {
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
int MotorControl::PILoop(int target_speed) {
  
  deadZone = 0.003; // for 10% motor power, the power level at witch the motor no longer turns //! move to befor ramp

  if (abs(target_speed) <= Percent2RPM(deadZone)) { // the motor wants to stop, skip and reset the PI loop  
    adjusted_speed = 0;
    integrateReset();
  } else {
    error = target_speed - getCurrentSpeed();
    
    adjusted_speed = k_p*error + k_i*integrate(error);

  }
  //Serial.println(adjusted_speed);

  return adjusted_speed; 

}

int MotorControl::Percent2RPM(float pct) {
   float temp = constrain(pct, -1, 1);
  return this->max_rpm * constrain(pct, -1.0f, 1.0f);
  //return copysign(4651*pow(abs(pct), 1.7783f), pct);
}

float MotorControl::RPM2Percent(int rpm) {
  int temp = constrain(rpm, -this->max_rpm, this->max_rpm);
  if (rpm == 0)
    return 0.0f; 
  return constrain(rpm, -this->max_rpm, this->max_rpm) / float(this->max_rpm);
  // if (rpm < 0)
  //   return copysign(.0012f*pow(constrain(abs(rpm), -this->max_rpm, this->max_rpm), 0.7895f), rpm);

  // return copysign(.0087f*pow(constrain(abs(rpm), -this->max_rpm, this->max_rpm), 0.5616f), rpm);
}

/**
 * @brief ramp slowly increases the motor power each iteration of the main loop,
 * this function is critical in ensuring the bot has proper traction with the floor,
 * think of it as the slope y=mx+b
 *
 * FUTURE: none...this is perfection (...atm...:0 )
 *
 * @authors Grant Brautigam, Julia DeVore, Lena Frate
 * Created: fall 2023
 *
 * @param requestedPower, accelRate
 * @return int
 */
int MotorControl::ramp(int requestedPower,  float accelRate) {
    timeElapsed = (millis() - lastRampTime)*.001;
    int abs_requestedPower = abs(requestedPower);
    // Serial.print("  time elapsed: ");
    // Serial.print(timeElapsed);

    // Serial.print("  acceleration: ");
    // Serial.print(accelRate);

    // Serial.print("  requested power: ");
    // Serial.print(requestedPower);

    // Serial.print("  currentPower: ");
    // Serial.print(currentPower);

    // Serial.print("\n");

    lastRampTime = millis();
    if (abs_requestedPower > requestedRPM) // need to speed up
    {
        requestedRPM = requestedRPM + accelRate * timeElapsed;
        if (requestedRPM > abs_requestedPower) 
            requestedRPM = abs_requestedPower; // to prevent you from speeding up past the requested speed
    }
    else // need to slow down
    {
        requestedRPM = requestedRPM - accelRate * timeElapsed *2; 
        if (requestedRPM < abs_requestedPower) 
            requestedRPM = abs_requestedPower; // to prevent you from slowing down below the requested speed
    }
    
    return copysign(requestedRPM, requestedPower);

}