#include "QuarterbackBase.h"

QuarterbackBase::QuarterbackBase(Drive* drive) {
    this->drive = drive;

    // Setup digital WiFi pin
    pinMode(WIFI_PIN, OUTPUT);
}

void QuarterbackBase::action() {
// Send data to wifi ESP
  updateWriteMotorValues();
}

void QuarterbackBase::updateWriteMotorValues() {
    targetValue = checkGetNewTarget();
    currentUpdateMotorMillis = millis();
    if (timesSentSession < targetValue && (currentUpdateMotorMillis-previousMillis)>50) {
      digitalWrite(WIFI_PIN, HIGH);
      previousMillis = currentUpdateMotorMillis;
      timesSentSession++;
      //Serial.println("Write High");
    } else if ((currentUpdateMotorMillis - previousMillis)>200) {
      timesSentSession = 0;
      previousMillis = currentUpdateMotorMillis;
    } else if ((currentUpdateMotorMillis-previousMillis) > 25) {
      digitalWrite(WIFI_PIN, LOW);
      //Serial.println("Write Low");
    } 
    /*
    Serial.print("Times Sent This Session: ");
    Serial.print(timesSentSession);
    Serial.print("\tTimeStep: ");
    Serial.print((currentMillis-previousMillis));
    Serial.println();
    */
}

int QuarterbackBase::checkGetNewTarget() {
  //This should only get updated every 350 millis
  unsigned long currentUpdateValueMillis = millis();
  if (((currentUpdateValueMillis - prevUpdateTargetMillis)) > 350) {
    prevUpdateTargetMillis = currentUpdateValueMillis;
    return testAnalogOutput;
  }
  return targetValue;
}