#pragma once

#include <Arduino.h>
#include <PolarRobotics.h>
// #include <MotorInterface.h>
#include <MotorInterface.h>


// Enum for Increasing or Decreasing Flywheel Speed
enum SpeedStatus {
  INCREASE, DECREASE
};

class MotorControl {
private:
  MotorType motor_type; // the type of motor to be assigned to this object
  float gear_ratio;     // the input / output gear ratio

  // Servo:
  MotorInterface Motor;

  // for ramp
  float requestedRPM;     
  float lastRampTime;
  float timeElapsed;

  // Encoder
  bool has_encoder;
  uint8_t encoderIndex;
  uint8_t enc_a_pin, enc_b_pin;

  // for use in void readEncoder()
  int encoderACount;
  int b_channel_state;
  int rollover;

  // For use in int calcSpeed()
  int prev_current_count;
  int rollover_threshold;
  unsigned long current_time;
  unsigned long prev_current_time;
  float omega;

  //PILoop
  int adjusted_speed;
  int error;
  float k_p;
  float k_i;
  float deadZone; //needs to be peramiterized

  //setTargetSpeed
  int ramped_speed;
  int set_speed;
  bool CL_enable;

  //getCurrentSpeed
  int current_speed;

  //integrate
  int prev_current_error;
  int integral_sum;
  unsigned long prev_integral_time;



public:
  int max_rpm;          // the motor max rpm * the gear ratio 
  MotorControl();
  uint8_t setup(int mot_pin, MotorType type = big_ampflow, bool has_encoder = false, float gearRatio = 1, int enc_a_chan_pin = -1, int enc_b_chan_pin = -1); // if no encoder, leave blank, will not attach pins
  uint8_t attach(int mot_pin, int min = MIN_PWM_US, int max = MAX_PWM_US); // as above but also sets min and max values for writes. 
  void displayPinInfo();
  
  void write(float pwr);
  void writelow();
  void stop();
  void setTargetSpeed(int target_rpm);
  void setCurrentSpeed(int speed);
  // Encoder Related Functions
  void readEncoder();
  int calcSpeed(int current_count);

  int Percent2RPM(float pct);
  float RPM2Percent(int rpm);

  int ramp(int requestedPower, float accelRate);

  // Encoder Related Functions
  void readEncoder();
  int calcSpeed(int current_count);

  // Closed Loop related functions
  int PILoop(int target_speed);
  int getCurrentSpeed();
  int trapIntagral(int current_error);
  int integrate(int current_error);
  void integrateReset();
};
