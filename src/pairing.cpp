#include <Arduino.h>
#include <SPI.h>
#include <PS5BT.h>

#define buttonPin 8

// The variables for PS5 and pair button
bool debounce = false;
bool usbConnected = false;

// the USB Host shield uses pins 9 through 13, so dont use these pins
USB Usb;            // There is a USB port
BTD Btd(&Usb);      // The Location of the Bluetooth port
PS5BT PS5(&Btd, 1);

void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200);
    Serial.print(F("\r\nStarting..."));
    
    // clear previously paired devices 
    // 0x41 is the address for all devices in the pool
    // Usb.GetAddressPool().FreeAddress(0x41);

    if (Usb.Init() == -1) {
        Serial.print(F("\r\nConnect USB... Again..."));
        while (Usb.Init() == -1) { // wait until the controller connects
            delay(5);
        }
    }
    else
        Serial.print(F("\r\nUSB Connected"));

    // Btd.hci_reset();
    // PS5BT PS5(&Btd, 1);
    // stop looking for other bluetooth devices
    // Btd.hci_inquiry_cancel(); 

    pinMode(buttonPin, INPUT);

    delay(1000);

    while(true) {
        // put your main code here, to run repeatedly:

        Usb.Task();

        if(PS5.connected()) {
            // Put code here to test the controller when connected
            Serial.print(F("\r\nLeftHatY: "));
            Serial.print(PS5.getAnalogHat(LeftHatY));
            Serial.print(F("RightHatX: "));
            Serial.print(PS5.getAnalogHat(RightHatX));

            if (PS5.getButtonClick(TRIANGLE)) {
                PS5.setRumbleOn(RumbleLow);
            }
            if (PS5.getButtonClick(CIRCLE)) {
                PS5.setRumbleOn(RumbleHigh);
            }
            if (PS5.getButtonClick(SQUARE)) {
                PS5.setRumbleOff();
            }

            if(PS5.getAnalogHat(LeftHatX) > 137 || PS5.getAnalogHat(LeftHatX) < 117 || 
            PS5.getAnalogHat(LeftHatY) > 137 || PS5.getAnalogHat(LeftHatY) < 117 || 
            PS5.getAnalogHat(RightHatX) > 137 || PS5.getAnalogHat(RightHatX) < 117 || 
            PS5.getAnalogHat(RightHatY) > 137 || PS5.getAnalogHat(RightHatY) < 117) {
            PS5.setLed(PS5.getAnalogHat(LeftHatY), PS5.getAnalogHat(LeftHatX), PS5.getAnalogHat(RightHatX));
            }
        }
    }
}

void loop() {
    
}

// if the controller is not connected and needs to be connected
// If the button is pressed and it is not debounced then go into statement
// if (digitalRead(buttonPin) == 0 && !debounce) {
//   // digitalWrite(LED_BUILTIN, HIGH);
//   Serial.println(F("Pairing..."));
//   debounce = true;             
//   PS5.disconnect();            // Disconnect the current PS5 controller
//   delete [] &PS5;              // Deletes the memory allocation for the PS5 controller so a new one can be created with same name
//   PS5 = PS5BT(&Btd, 1);        // Re-initalizes the PS5 object
//   do {                         // Delay any other code from running until the PS5 controller is connected
//     delay(10);
//   } while (!PS5.connected());
//   if(PS5.connected()) {        // Reset the debounce when it finally connects so button can be pressed and it can run again
//     // digitalWrite(LED_BUILTIN, LOW);
//     debounce = false;
//   }
// }