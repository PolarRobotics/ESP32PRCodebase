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
        double currentHeading = 0; // Current heading of the robot in radians
        const int RADIUS = 1.925; // Radius of the wheel in inches
        const double WHEEL_BASE = 9.5/12; // Distance between the wheels in feet
        double circumference = 2*PI*RADIUS/12; // Circumference of the wheel in feet
        double velocity; // Speed in feet per second
        double omega; // Angular velocity in radians per second
        double turningRadius; // Turning radius in feet
        double d1, d2; // Distances traveled by the wheels in feet
        double deltaTheta = 0; // Change in heading in radians
        bool newData = false;
        char receivedChars[NUM_CHARS];
        char tempChars[NUM_CHARS];
        char sendingData[NUM_CHARS];
        HardwareSerial encSerial = Serial2;

    public:
        Encoder(int baud, int numEncoders);
        void readData();
        void parseData();
        void sendData(int x);
        void recvDataWithMarkers();
        double calcDistance(int encNum);
        double calcVelocity(int encNum);
        double calcOmega();
        double calcTurningRadius();
        double calcHeading();
        void updatePosition();
        int getCounts(int encNum);
        int getRPM(int encNum);
        double* getCurrentPos();
        double* getTargetPos();
        void updateTurret();

};