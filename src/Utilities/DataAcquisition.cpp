#include <Arduino.h>
// #include <Preferences.h>
// #include <PolarRobotics.h>
#include <Utilities/ConfigManager.h>

int ascendingNumber = 0;

void setup() {
    Serial.begin(115200);
}
    // Insert Script to send data from sensors to Raspberry Pi here!
void loop() {
    Serial.print("Ascending number = ");
    Serial.println(ascendingNumber);
    ascendingNumber++;
    delay(500);
}