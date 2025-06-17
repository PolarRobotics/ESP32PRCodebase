#include "Encoder.h"

Encoder::Encoder(int baud, int rx, int tx, int numEncoders){
    encSerial.begin(baud);
    encSerial.setPins(rx,tx,-1,-1);
    this->numEncoders = numEncoders;
    this->data = new encoderData[numEncoders];
}

void Encoder::readData(){
    recvDataWithMarkers();
    if(newData == true){
        strcpy(tempChars,receivedChars);
        parseData();
        newData = false;
    }
}

void Encoder::recvDataWithMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;
    
    while (encSerial.available() > 0 && newData == false) {
        // Serial.println("Data Received");
        rc = encSerial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= NUM_CHARS) {
                    ndx = NUM_CHARS - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }
        
        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

void Encoder::parseData() {      // split the data into its parts
    char * strtokIndx; // this is used by strtok() as an index
    // Serial.println("test");
    strtokIndx = strtok(tempChars,",");

    for(int i = 0; i < numEncoders; i++){
        if(strtokIndx != NULL){
            data[i].counts = atoi(strtokIndx);
            
        }
        strtokIndx = strtok(NULL, ",");
        if(strtokIndx != NULL){
            data[i].speed = atoi(strtokIndx);
        }
        strtokIndx = strtok(NULL, ",");
    }
}

int Encoder::getCounts(int encNum){
    return data[encNum].counts;
}

int Encoder::getSpeed(int encNum){
    return data[encNum].speed;
}