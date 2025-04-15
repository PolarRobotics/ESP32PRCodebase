#include <Arduino.h>

// template <typename TYPE>
class PID {
private:
  // "Constants"
  float k_p;
  float k_i;
  float k_d;
  float error_threshold;
  float output_minmax[2]; //sets bounds on the output of the controller

  // Inputs
  float measured_value; //measured data from the sensor
  float set_point; // commanded value for the controller to try and match
  bool cl_enable; // enable or disables the CL calculations
  
  // Outputs
  float controller_output; // the calculated PID value to be sent to the plant
  float error; // difference between commanded value and actual measured value
  
  // Integration
  float prev_error;
  float integral_sum;
  unsigned long prev_integral_time;
  
  float integrate(float current_error);
  void integralReset();
  float differentiate(float current_error);
public:
  PID();
  PID(float k_p, float k_i, float k_d = 0.0f, float error_thresh = 0.2f, 
    float output_min = 0.0f, float output_max = 1.0f);
  void setCLState(bool state);
  void setMeasuredValue(float measured_value);
  float PIDLoop(float setpoint);
};
