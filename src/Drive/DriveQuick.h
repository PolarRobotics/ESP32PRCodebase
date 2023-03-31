#include <Drive/Drive.h>

// the minimum power that can be written to the motor, prevents stalling
#define MOTOR_ZERO_OFFST 0.05

class DriveQuick : public Drive {
    private:
        float falcon_motor_pwr[NUM_MOTORS];
        float r, falconTurnPwr, max;
    public:
        void generateMotionValues();
        void update();
};