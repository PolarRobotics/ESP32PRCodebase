#include <Arduino.h>
#include <EEPROM.h>

int robotTyp = 0;
int robotAge = 0;

void setup() {

  Serial.begin(9600);
  Serial.println("Reading");
  robotAge = EEPROM.read(0);
  Serial.println(robotAge);
  // 0 = old robot
  // 1 = new robot
  robotTyp = EEPROM.read(1);
  Serial.println(robotTyp);
  // 0 = Lineman
  // 1 = Recever
  // 2 = Center
  // 3 = Quarterback
  // 4 = Kicker
  Serial.println(EEPROM.read(2));
  Serial.println("Done");
}

void loop() {

}
