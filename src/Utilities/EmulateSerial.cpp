// A very simple script for Data Aq to emulate expected
// serial output from ESP32, without controller input necessary

#include <PolarRobotics.h>

void setup() {
    Serial.begin(115200);
}
void loop() {
    Serial.print(F("L_HAT_Y,0.00,R_HAT_X,0.00,Turn,0.46,Left Motor,0.00,Right Motor,0.00\n"));
}