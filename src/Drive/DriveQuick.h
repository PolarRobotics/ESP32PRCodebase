#ifndef DRIVE_QUICK_H
#define DRIVE_QUICK_H
// the minimum power that can be written to the motor, prevents stalling
#define MOTOR_ZERO_OFFST 0.05
#define RB_ACCELERATION_RATE 0.0015f //default: 0.00375f, Runningback old: 0.03f, 0.015f
#include <Drive/Drive.h>

class DriveQuick : public Drive {
  private:
    // float falcon_motor_pwr[NUM_MOTORS];
    float r, falconTurnPwr, max;
  public:
    //! Must call base class constructor with appropriate arguments
    DriveQuick(drive_param_t driveParams); // This is the prototype, see DriveQuick.cpp for implementation
    // void generateMotionValues();
    void update() override;
};

#endif // DRIVE_QUICK_H