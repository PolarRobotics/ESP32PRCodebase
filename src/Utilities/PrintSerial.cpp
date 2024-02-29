#include <Arduino.h>
#include <Drive/Drive.h>
#include "PrintSerial.h"

PrintSerial::PrintSerial() {
    PrintSerial::serialHeaders = {"header1","header2","header3","header4","header5"};
    PrintSerial::serialValues = {1.0, 2.0, 3.0, 4.0, 5.0};
}

/**
 * @brief prints the internal variables to the serial monitor in a clean and easy to read format
 * @author Everybody
 * @date 2024-02-12
*/
void PrintSerial::printDebugInfo() {
    // Serial.print(F("L_Hat_Y: "));
    // Serial.print(stickForwardRev);
    // Serial.print(F("  R_HAT_X: "));
    // Serial.print(stickTurn);

    // Serial.print(F("  |  Turn: "));
    // Serial.print(lastTurnPwr);

    // Serial.print(F("  |  Left ReqPwr: "));
    // Serial.print(requestedMotorPower[0]);
    // Serial.print(F("  Right ReqPwr: "));
    // Serial.print(requestedMotorPower[1]);

    // Serial.print(F("  |  Omega: "));
    // Serial.print(omega);

    // Serial.print(F("  omega_L: "));
    // Serial.print(omega_L);
    // Serial.print(F("  omega_R: "));
    // Serial.print(omega_R);

    // Serial.print(F("  lastRampTime "));
    // Serial.print(lastRampTime[0]);
    // Serial.print(F("  requestedPower "));
    // Serial.print(requestedPower);
    // Serial.print(F("  current "));
    // Serial.print(currentRampPower[0]);
    // Serial.print(F("  requestedPower - currentRampPower "));
    // Serial.println(requestedPower - currentRampPower[mtr], 10);

    // Serial.print(F("  Left Motor: "));
    // Serial.print(requestedMotorPower[0]);
    // Serial.print(F("  Right: "));
    // Serial.print(requestedMotorPower[1]);

    //Serial.print(F("  scaledSensitiveTurn: "));
    //Serial.print(scaledSensitiveTurn);

    // Serial.print(F("\n"));
}

/**
 * @brief Prints variables to the serial monitor in a csv format
 * This function is important for data acquisition
 * The options below are configurable, change them as you need
 * Remember to adhere to printing guidelines under PR-Docs
 * @param values A vector of float values that will be sent to serial monitor 
 * @param headers A vector of header strings that will be sent to serial monitor
 * @author Corbin Hibler
 * Updated: 2024-02-12
*/
void PrintSerial::printCsvInfo(const std::vector<float>& values, const std::vector<String>& headers) {
    for (int i = 0; i < serialValues.size(); i++) {
        if (i == 0) {
            String header = serialHeaders[i] + ","; 
            Serial.print(header.c_str());
            Serial.print(serialValues[i]);
        }
        else if (i < (serialValues.size() - 1)) {
            String header = "," + serialHeaders[i] + ",";
            Serial.print(header.c_str());
            Serial.print(serialValues[i]);
        }
        else {
            String header = "," + serialHeaders[i] + ",";
            Serial.print(header.c_str());
            Serial.println(serialValues[i]);
        }
    }
}