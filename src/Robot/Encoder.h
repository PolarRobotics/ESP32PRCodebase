#include <Arduino.h>

#define NUM_CHARS 100

class Encoder{
    private:
        struct encoderData{
            int counts = 0;
            int speed = 0;
        };
        encoderData* data;
        int numEncoders;
        bool newData = false;
        char receivedChars[NUM_CHARS];
        char tempChars[NUM_CHARS];

    public:
        Encoder(int baud, int rx, int tx, int numEncoders);
        void readData();
        void parseData();
        void recvDataWithMarkers();
        int getCounts(int encNum);
        int getSpeed(int encNum);
        HardwareSerial encSerial = Serial2;
};