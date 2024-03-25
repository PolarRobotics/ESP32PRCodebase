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
  float deadZone = 0.001;

  // Servo:
  MotorInterface Motor;

  // for ramp
  float requestedRPM;     
  float lastRampTime;
  float timeElapsed;

  // for sendRPM
  coeffs_t coeff;
  bool negativeDir = false;
  float pct = 0;

public:
  int max_rpm;          // the motor max rpm * the gear ratio 
  MotorControl();
  uint8_t setup(int mot_pin, MotorType type = big_ampflow, float gearRatio = 1); 

  //! TEMPORARY FUNCTION, TO BE REMOVED IN FUTURE
  void write(float pct);

  void sendRPM(int rpm);

  int Percent2RPM(float pct);
  float RPM2Percent(int rpm);

  float ramp(float requestedPower, float accelRate);

  void setTargetSpeed(int target_rpm);

};