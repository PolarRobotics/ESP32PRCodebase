#include "QuarterbackBase.h"

// This for some reason has to be declared in the .cpp file and not the .h file 
// so that it does not conflict with the same declaration in other .h files
HardwareSerial qbBaseUART(2);     // UART2

QuarterbackBase::QuarterbackBase(Drive* drive) {
    this->drive = drive;

    // Setup digital WiFi pin
    pinMode(WIFI_PIN, OUTPUT);

    //Setup UART Pins
    qbBaseUART.begin(115200, SERIAL_8N1, UART_RX2, UART_TX2);
}

void QuarterbackBase::action() {
// Send data to wifi ESP
  updateWriteMotorValues();
}

void QuarterbackBase::updateWriteMotorValues() {
    motor1Value = drive->getMotorWifiValue(0);
    motor2Value = drive->getMotorWifiValue(1);
    UARTMessage = String(motor1Value) + "&" + String(motor2Value);
    UARTMessage = UARTMessage + "~";
    qbBaseUART.print(UARTMessage);
    //Serial.print("Sent Message To ESP: ");
    Serial.println(UARTMessage);
}