#include <Arduino.h>
// the minimum power that can be written to the motor, prevents stalling
#define MOTOR_ZERO_OFFST 0
#define ACCELERATION_RATE 0.000001f
#include <Drive/Drive.h>

class DriveQuick : public Drive {
    private:
        float falcon_motor_pwr[NUM_MOTORS];
        float r, falconTurnPwr, max;
    public:
        // void generateMotionValues();
        void update();
};