#pragma once

#include <Arduino.h>
#include <PolarRobotics.h>
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

public:
  int max_rpm;          // the motor max rpm * the gear ratio 
  MotorControl();
  uint8_t setup(int mot_pin, MotorType type = big_ampflow, bool has_encoder = false, float gearRatio = 1, int enc_a_chan_pin = -1, int enc_b_chan_pin = -1); // if no encoder, leave blank, will not attach pins

  //! TEMPORARY FUNCTION, TO BE REMOVED IN FUTURE
  void write(float pct);

  int Percent2RPM(float pct);
  float RPM2Percent(int rpm);

  float ramp(float requestedPower, float accelRate);
};
