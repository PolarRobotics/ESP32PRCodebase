#include <Arduino.h>
#include "Utilities/PID.h"

// Public methods
PID::PID() {
  this->k_p = 0;
  this->k_i = 0;
  this->k_d = 0;
  this->error_threshold = 0;
  output_minmax[0] = 0;
  output_minmax[1] = 0;
}

PID::PID(float k_p, float k_i, float k_d, float error_thresh,
  float output_min, float output_max)
{
  this->k_p = k_p;
  this->k_i = k_i;
  this->k_d = k_d;
  this->error_threshold = error_thresh;

  // clamp the output limits of the controller
  if (output_min > output_max) {
    Serial.println(F("Utilities/PID.cpp: min larger than max value..."));
    output_minmax[0] = output_max;
    output_minmax[1] = output_min;
  } else {
    output_minmax[0] = output_min;
    output_minmax[1] = output_max;
  }
}

void PID::setCLState(bool state) {
  this->cl_enable = state;
}

void PID::setMeasuredValue(float measured_value) {
  this->measured_value = measured_value;
}

/**
 * PIDLoop is the closed loop controller. this is the main function for CL
 * 
 * @authors Grant Brautigam, Rhys Davies
 * Created: 11-19-2023
 * Updated: 2-5-2025
 */
float PID::PIDLoop(float setpoint) {
  error = setpoint - measured_value;
  if (fabs(error) >= error_threshold) { // the motor wants to stop, skip and reset the PI loop  
    controller_output = k_p * error +               // proportional
                        k_i * integrate(error) +    // integral
                        k_d * differentiate(error); // derivitave
    // Serial.println(motorDiffCorrection);
  } else {
    controller_output = 0;
    integralReset();
  }

  return constrain(controller_output, output_minmax[0], output_minmax[1]);
}

// Private methods:

/**
 * integrate uses trapizodial intgration to calculate the running integral sum for the PI controller
 * @authors Grant Brautigam, Rhys Davies
 * Created: 11-19-2023
 * Updated: 2-5-2025
 * 
 * @param current_error
 * @return returns the integral sum of all current and previous values
 */
float PID::integrate(float current_error) {
  integral_sum = integral_sum + (current_error + prev_error); //*(millis()-prev_integral_time)/100;
  prev_integral_time = millis();
  prev_error = current_error;
  
  return integral_sum; 
}

void PID::integralReset() {
  integral_sum = 0;
  prev_error = 0;
  prev_integral_time = millis();
}

float PID::differentiate(float current_error) {
  return current_error - prev_error;
}
