#ifndef DRIVE_QUICK_H
#define DRIVE_QUICK_H
// the minimum power that can be written to the motor, prevents stalling
#define MOTOR_ZERO_OFFST 0.05
#define RB_ACCELERATION_RATE 0.02f //default: 0.0375f, Runningback old: 0.03f, 0.015f
#include <Drive/Drive.h>

class DriveQuick : public Drive {
  private:
    float falcon_motor_pwr[NUM_MOTORS];
    float r, falconTurnPwr, max;
  public:
    // void generateMotionValues();
    void update() override;
    void printDebugInfo() override;
};

#endif // DRIVE_QUICK_H