#include "Encoder.h"

/**
 * @brief Encoder Class Implementation. This class handles reading the encoders from the Quarterback base.
 * The encoders are read by the using the raspberry pi pico and sent to the base via UART.
 * The encoder data is then used to calculate the position of the base, and the relative angle of the turret.
 * The primary goal of this class is to allow the Quarterback turret to remain focused on the target position while the base moves.
 * @author Kaiden Colish
 * @date 6-23-2025
 */
Encoder::Encoder(int baud, int numEncoders){
    encSerial.begin(baud);
    encSerial.setPins(16,17,-1,-1); // RX on GPIO 16, TX on GPIO 17, no RTS or CTS
    this->numEncoders = numEncoders;
    this->data = new encoderData[numEncoders];
}

/**
 * @brief Reads the data from the encoders and parses it for calculating position.
 */
void Encoder::readData(){
    recvDataWithMarkers();
    if(newData == true){
        strcpy(tempChars,receivedChars);
        parseData();
        newData = false;
    }
}

/**
 * @brief Helper function for readData() that reads the uart data with start and end markers.
 * The data is expected to be in the format <counts1,RPM1,counts2,RPM2>
 */
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

void Encoder::sendData(int x){
    snprintf(sendingData, NUM_CHARS, "%d", x);
    encSerial.print("<");
    encSerial.print(sendingData);
    encSerial.print(">");
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
    return data[encNum].counts * circumference / 4000; // Distance in feet
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

double Encoder::calcHeading(){
    // Calculate the new heading based on the wheel distances
    d1 = calcDistance(0);
    d2 = calcDistance(1);
    deltaTheta = (d2 - d1) / (WHEEL_BASE/2);
    return deltaTheta;
}

void Encoder::updatePosition(){
    double hc = calcHeading();
    double d = (d1 + d2); // Average distance traveled by both wheels
    double x = d * cos(hc); // Change in x position
    double y = d * sin(hc); // Change in y position
    currentPos[0] = x; // Update x position
    currentPos[1] = y; // Update y position
    currentHeading = hc; // Change in heading
}

void Encoder::updatePositionICC(){
    if((abs(data[0].counts - prevCounts[0])) < 100 && (abs(data[1].counts - prevCounts[1])) < 100){
        // If large encoder change, reuse previous distance values
        d1 = (data[0].counts-prevCounts[0]) * circumference / 4000; // Distance traveled by left wheel in feet
        d2 = (data[1].counts-prevCounts[1]) * circumference / 4000; // Distance traveled by right wheel in feet

    }
    prevCounts[0] = data[0].counts; // Update previous counts for left wheel
    prevCounts[1] = data[1].counts; // Update previous counts for right wheel

    deltaTheta = (d1 - d2) / WHEEL_BASE; // Change in heading in radians
    if(deltaTheta > PI/2 || deltaTheta < -PI/2){
        return;
    }
    else if(abs(deltaTheta) < 0.0001){
        double d = (d1 + d2) / 2; // Average distance traveled by both wheels
        currentPos[0] += d * cos(currentHeading); // Update x position
        currentPos[1] += d * sin(currentHeading); // Update y position
    }
    else{
        double turningRadius = WHEEL_BASE / 2 * (d2 + d1) / (d2 - d1); // Turning radius in feet
        double ICCx = currentPos[0] - turningRadius * sin((PI/2) - currentHeading); // x coordinate of the Instantaneous Center of Curvature
        double ICCy = currentPos[1] + turningRadius * cos((PI/2) - currentHeading); // y coordinate of the Instantaneous Center of Curvature
        currentPos[0] = cos(deltaTheta) * (currentPos[0] - ICCx) - sin(deltaTheta) * (currentPos[0] - ICCx) + ICCx; // Update x position
        currentPos[1] = sin(deltaTheta) * (currentPos[1] - ICCy) + cos(deltaTheta) * (currentPos[1] - ICCy) + ICCy; // Update y position
        currentHeading += deltaTheta; // Update heading
        // Normalize heading to [0, 2*PI)
        while (currentHeading < 0) {
            currentHeading += 2 * PI;
        }
        while (currentHeading >= 2 * PI) {
            currentHeading -= 2 * PI;
        }
    }
}

void Encoder::updateTurret(){
    // updatePosition(); // Update the robot's position based on encoder data
    updatePositionICC(); // Update the robot's position using the Instantaneous Center of Curvature method
    double dx = targetPos[0] - currentPos[0]; // Change in x position
    double dy = targetPos[1] - currentPos[1]; // Change in y position
    double distance = sqrt(dx*dx + dy*dy); // Distance to target
    double angle = atan2(dy, dx); // Angle to target
    double turretAngle = angle - (currentHeading); // Angle of turret relative to robot
    // Normalize turret angle to [0, 2*PI)
    while (turretAngle < 0) {
        turretAngle += 2 * PI;
    }
    while (turretAngle >= 2 * PI) {
        turretAngle -= 2 * PI;
    }
    Serial.printf("Turret Angle: %3d degrees\tCurrent Pos: x=%8f, y=%8f\tHeading: %d\tDelta Theta: %f\n", (int)(turretAngle * 180 / PI), currentPos[0], currentPos[1], (int)(currentHeading * 180 / PI),deltaTheta); // Print turret angle in degrees
    sendData((int)(turretAngle * 180 / PI)); // Send turret angle in degrees
}
    