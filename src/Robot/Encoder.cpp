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
            data[i].RPM = atoi(strtokIndx);
        }
        strtokIndx = strtok(NULL, ",");
    }
}

int Encoder::getCounts(int encNum){
    return data[encNum].counts;
}

int Encoder::getRPM(int encNum){
    return data[encNum].RPM;
}

double* Encoder::getCurrentPos(){
    // Return the current position of the robot
    return currentPos;
}

double* Encoder::getTargetPos(){
    // Return the target position of the robot
    return targetPos;
}

double Encoder::calcDistance(int encNum){
    // Calculate the distance traveled by the wheel based on the encoder counts
    return (data[encNum].counts / 4000.0) * circumference; // 4000 counts per revolution for the encoder
}

double Encoder::calcVelocity(int encNum){
    // Calculate the speed of the wheel based on the encoder speed
    return PI*RADIUS*data[encNum].RPM/30;
}

double Encoder::calcOmega(){
    // Calculate the angular velocity of the robot based on the wheel speeds
    double v1 = calcVelocity(0);
    double v2 = calcVelocity(1);
    omega = (v2 - v1) / WHEEL_BASE;
    return omega;
}

double Encoder::calcTurningRadius(){
    // Calculate the turning radius of the robot based on the wheel speeds
    double v1 = calcVelocity(0);
    double v2 = calcVelocity(1);
    if(v2 - v1 == 0) return 0; // Avoid division by zero
    return WHEEL_BASE/2 * (v2 + v1) / (v2 - v1);
}

double Encoder::headingChange(){
    // Calculate the change in heading based on the wheel speeds
    d1 = calcDistance(0);
    d2 = calcDistance(1);
    deltaTheta = (d2 - d1) / WHEEL_BASE;
    return deltaTheta;
}

void Encoder::updatePosition(){
    double d = (d1 + d2) / 2; // Average distance traveled by both wheels
    prevHeading = currentHeading; // Store the previous heading
    double x = d * cos(currentHeading + headingChange()/2); // Change in x position
    double y = d * sin(currentHeading + headingChange()/2); // Change in y position
    currentPos[0] += x; // Update x position
    currentPos[1] += y; // Update y position
    currentHeading += headingChange(); // Change in heading
}

void Encoder::updateTurret(){
    double dx = targetPos[0] - currentPos[0]; // Change in x position
    double dy = targetPos[1] - currentPos[1]; // Change in y position
    double distance = sqrt(dx*dx + dy*dy); // Distance to target
    double angle = atan2(dy, dx); // Angle to target
    double turretAngle = angle - currentHeading; // Angle of turret relative to robot
    if(turretAngle < -PI) turretAngle += PI; // Normalize angle to [-PI, PI]
    if(turretAngle > PI) turretAngle -= PI; // Normalize angle to [-PI, PI]
}
    