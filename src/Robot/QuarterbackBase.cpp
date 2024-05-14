#include "QuarterbackBase.h"

//This for some reason has to be declared in the .cpp file and not the .h file so that it does not conflict with the same declaration in other .h files
HardwareSerial Uart_Base(2);     // UART2

QuarterbackBase::QuarterbackBase(Drive* drive) {
    this->drive = drive;

    // Setup digital WiFi pin
    pinMode(WIFI_PIN, OUTPUT);

    //Setup UART Pins
    Uart_Base.begin(115200, SERIAL_8N1, RX2, TX2);
}

void QuarterbackBase::action() {
// Send data to wifi ESP
  updateWriteMotorValues();
}

void QuarterbackBase::updateWriteMotorValues() {
    targetValue = drive->getMotorWifiValue(0);
    UARTMessage = String(targetValue);
    Uart_Base.print(UARTMessage);
}