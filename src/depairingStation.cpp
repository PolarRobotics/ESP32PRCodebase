// #include <Arduino.h>
#include <ps5Controller.h> // new esp ps5 library

// Custom Polar Robotics Libraries:
#include "PolarRobotics.h"
#include "pairing.h"
#include <builtinLED.h>


// Lights robotLED;


/**
 * @brief onConnection: Function to be called on controller connect
 */
void onConnection() {
    if(ps5.isConnected()) {
        Serial.println(F("Controller Connected."));
        ps5.setLed(0, 255, 0);   // set LED green
    }
}

/**
 * @brief onDisconnect: Function to be called on controller disconnect
 */
void onDisconnect() {
    Serial.println(F("Controller Disconnected."));
}

/*
   ____    _____   _____   _   _   ____
  / ___|  | ____| |_   _| | | | | |  _ \
  \___ \  |  _|     | |   | | | | | |_) |
   ___) | | |___    | |   | |_| | |  __/
  |____/  |_____|   |_|    \___/  |_|

*/

// ESP32PWM pwm;

void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    activatePairing(false, 1000000);

    // Serial.print(F("\r\nConnected"));

    // ps5.attachOnConnect(onConnection);
    ps5.attachOnDisconnect(onDisconnect);
}

/*
   __  __      _      ___   _   _     _        ___     ___    ____
  |  \/  |    / \    |_ _| | \ | |   | |      / _ \   / _ \  |  _ \
  | |\/| |   / _ \    | |  |  \| |   | |     | | | | | | | | | |_) |
  | |  | |  / ___ \   | |  | |\  |   | |___  | |_| | | |_| | |  __/
  |_|  |_| /_/   \_\ |___| |_| \_|   |_____|  \___/   \___/  |_|

*/
void loop() {
    if (ps5.isConnected()) {
        // ps5.setLed(255, 0, 0);   // set LED red
        // for debugging connection
        if (ps5.Square() || ps5.Circle() || ps5.Cross() || ps5.Triangle() ||
            ps5.L1() || ps5.L2() || ps5.R1() || ps5.R2() || ps5.Touchpad()) {
            Serial.println(F("PS5 pressed"));
        }
        if (!builtInLedOn()) digitalWrite(LED_BUILTIN, HIGH);
        delay(20);
    } else {
      Serial.println(F("PS5 controller not connected!"));
      toggleBuiltInLED();
      delay(1000);
    }

}
