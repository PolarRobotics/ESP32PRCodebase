/**
 * Bluetooth Classic Example
 * Scan for devices - asynchronously, print device as soon as found
 * query devices for SPP - SDP profile
 * connect to first device offering a SPP connection
 * 
 * Example python server:
 * source: https://gist.github.com/ukBaz/217875c83c2535d22a16ba38fc8f2a91
 *
 * Tested with Raspberry Pi onboard Wifi/BT, USB BT 4.0 dongles, USB BT 1.1 dongles, 
 * 202202: does NOT work with USB BT 2.0 dongles when esp32 aduino lib is compiled with SSP support!
 *         see https://github.com/espressif/esp-idf/issues/8394
 *         
 * use ESP_SPP_SEC_ENCRYPT|ESP_SPP_SEC_AUTHENTICATE in connect() if remote side requests 'RequireAuthentication': dbus.Boolean(True),
 * use ESP_SPP_SEC_NONE or ESP_SPP_SEC_ENCRYPT|ESP_SPP_SEC_AUTHENTICATE in connect() if remote side has Authentication: False
 */

// #include <Arduino.h>
#include <map>
#include <BluetoothSerial.h>
#include <ps5Controller.h>
#include "PolarRobotics.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

BluetoothSerial SerialBT;

#define BT_DISCOVER_TIME  20000
esp_spp_sec_t sec_mask = ESP_SPP_SEC_NONE; // or ESP_SPP_SEC_ENCRYPT|ESP_SPP_SEC_AUTHENTICATE to request pincode confirmation
esp_spp_role_t role = ESP_SPP_ROLE_SLAVE; // ESP_SPP_ROLE_MASTER or ESP_SPP_ROLE_SLAVE

// in the original code, this was all in the if condition... removed for legibility
bool startDiscovery() {
  return SerialBT.discoverAsync([](BTAdvertisedDevice* pDevice) {   
      Serial.print(F("Found a new device asynchronously: "));
      Serial.println(pDevice->toString().c_str());
    } 
  );
}

// bool isController(const char* name) {
//   const char* controller = "Wireless Controller";
//   int nlen = static_cast<int>(strlen(name));
//   int clen = static_cast<int>(strlen(controller));
//   Serial.print("length name = ");
//   Serial.print(nlen);
//   Serial.print(", length controller name = ");
//   Serial.println(clen);
//   if (nlen != clen) return false;
//   else {
//     Serial.println("name lengths equal, continuing...");
//     for (int i = 0; i < nlen; i++) {
//       Serial.print("name[i] = " + name[i]);
//       Serial.println("controller[i] = " + controller[i]);
//       if (name[i] != controller[i]) {
//         Serial.println("not equal, returning false");
//         return false;
//       }
//     }
//   }
//   Serial.println("equal, returning true");
//   return true;
// }

// Search for PS5 Controllers and pair to the first one found
void activatePairing() {
  Serial.begin(115200);

  // begin broadcasting as "ESP32" as master role
  if (!SerialBT.begin("ESP32", true)) { 
    Serial.println(F("SerialBT failed!")); // function returns false if failed
    abort();
  }
  SerialBT.enableSSP(); // according to SRC of this code, doesn't seem to change anything

  Serial.println(F("Searching for devices..."));
  BTScanResults* btDeviceList = SerialBT.getScanResults();  // may be accessing from different threads!

  if (startDiscovery()) {
    delay(BT_DISCOVER_TIME);
    Serial.println(F("Stopping discoverAsync... "));
    SerialBT.discoverAsyncStop();
    Serial.println(F("discoverAsync stopped"));
    delay(5000);
    if(btDeviceList->getCount() > 0) {
      BTAddress addr;
      int channel=0;
      Serial.println(F("Found devices:"));
      for (int i = 0; i < btDeviceList->getCount(); i++) {
        BTAdvertisedDevice* device = btDeviceList->getDevice(i);
        addr = device->getAddress();
        auto name = device->getName().c_str(); // get name to print and check
        auto addrStr = addr.toString().c_str(); // std::string doesn't work with Serial.print for some reason
        // ps5.begin requires a const char*, so get memory address of string/char array
        const char* addrCharPtr = &addr.toString().c_str()[0];

        // reminder that we need to use flash strings whenever possible, so don't try to collapse this
        Serial.print(i);
        Serial.print(F(" | "));
        Serial.print(addr.toString().c_str());
        Serial.print(F(" | "));
        Serial.print(device->getName().c_str());
        Serial.print(F(" | "));
        Serial.println(device->getRSSI());

        if (strcmp(device->getName().c_str(), "Wireless Controller") == 0) {
          Serial.print(F("Connecting to PS5 Controller @ "));
          Serial.println(addrCharPtr);
          ps5.begin(addrCharPtr);
          while (!ps5.isConnected()) {
            delay(100);
          }
          Serial.print(F("PS5 Controller Connected: "));
          Serial.println(ps5.isConnected());
        }
      }
    } else {
      Serial.println(F("Found no pairable devices."));
    }
  } else {
    Serial.println(F("Asynchronous discovery failed."));
  }
}