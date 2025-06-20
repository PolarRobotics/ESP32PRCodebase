#include <Arduino.h>
#include <math.h>

#define NUM_CHARS 100

class Encoder{
    private:
        struct encoderData{
            int counts = 0;
            int RPM = 0;
        };
        encoderData* data;
        int numEncoders;
        double startPos[2] = {0,0}; // Starting position of the robot
        double currentPos[2] = {0,0}; // Current position of the robot
        double targetPos[2] = {0,6}; // Position of target
        double currentHeading = 90; // Current heading of the robot in radians
        double prevHeading = 90;
        const int RADIUS = 1.925; // Radius of the wheel in inches
        const double WHEEL_BASE = 11.5/12; // Distance between the wheels in feet
        double circumference = 2*PI*RADIUS/12; // Circumference of the wheel in feet
        double velocity; // Speed in feet per second
        double omega; // Angular velocity in radians per second
        double turningRadius; // Turning radius in feet
        double d1, d2; // Distances traveled by the wheels in feet
        double deltaTheta; // Change in heading in radians
        bool newData = false;
        char receivedChars[NUM_CHARS];
        char tempChars[NUM_CHARS];
        HardwareSerial encSerial = Serial2;

    public:
        Encoder(int baud, int rx, int tx, int numEncoders);
        void readData();
        void parseData();
        void recvDataWithMarkers();
        double calcDistance(int encNum);
        double calcVelocity(int encNum);
        double calcOmega();
        double calcTurningRadius();
        double headingChange();
        void updatePosition();
        int getCounts(int encNum);
        int getRPM(int encNum);
        double* getCurrentPos();
        double* getTargetPos();
        void updateTurret();
};