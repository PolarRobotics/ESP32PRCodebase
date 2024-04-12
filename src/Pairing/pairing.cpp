/**
 * @brief Implements pairing functions.
 * @author Max Phillips
 * Adapted from: https://github.com/espressif/arduino-esp32/blob/master/libraries/BluetoothSerial/examples/DiscoverConnect/DiscoverConnect.ino
 * 
 * Provides a framework to handle pairing of an ESP32 to a PS5 controller.
 * `activatePairing()` should be called in your main code file during setup.
 * It is a *blocking* function (not asynchronous and takes a discrete amount of time), so consider that.
 * Other functions in this file are helper functions for `activatePairing()` and generally should not be called outside it.
 */

/**
 * [Source Comment]
 * 
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

#include <map>
#include <BluetoothSerial.h>
#include <ps5Controller.h>
#include <Preferences.h> // to store address of controller on flash
#include "Pairing/pairing.h" // also includes PolarRobotics.h
#include <Robot/builtInLED.h> // pairing routine flashes LED to signify stages of pairing
#include <Robot/Lights.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

#define yeet return

#define PREF_KEY "bt-mac" // preferences namespace, limited to 15 characters
Preferences prefs;

BluetoothSerial SerialBT;

#define LOOP_DELAY 100

bool foundController = false;

// Bluetooth connection security and role for ESP32
esp_spp_sec_t sec_mask = ESP_SPP_SEC_NONE; // or ESP_SPP_SEC_ENCRYPT|ESP_SPP_SEC_AUTHENTICATE to request pincode confirmation
esp_spp_role_t role = ESP_SPP_ROLE_SLAVE; // ESP_SPP_ROLE_MASTER or ESP_SPP_ROLE_SLAVE

// MAC Addresses to match to PS5 Controllers
const char* macTest = "bc:c7:46:03"; // length 11
const char* macTest2 = "bc:c7:46:04"; // length 11
const char* RhysController = "10:18:49:57:49:ef"; // length 17

/// @brief Detects if a given MAC Address is considered a PS5 Controller
/// @param addrCharPtr the address to test (C string)
/// @return true if the address represents a controller, false otherwise
bool addressIsController(const char* addrCharPtr) {
  if (strncmp(addrCharPtr, macTest, 11) == 0)
    return true;
  else if (strncmp(addrCharPtr, macTest2, 11) == 0)
    return true;
  else if (strncmp(addrCharPtr, RhysController, 17) == 0)
    return true;
  else return false;
}

/// @brief Begins the asynchronous discovery process for PS5 controllers
/// @return a boolean if the discovery started successfully. should generally return true.
/// This function is used in `activatePairing()` to begin async discovery if a paired controller is not found
bool startDiscovery() {
  return SerialBT.discoverAsync([](BTAdvertisedDevice* pDevice) {   
      Serial.print(F("Found a new device asynchronously: "));
      Serial.println(pDevice->toString().c_str());

      // Tests if the address of the device found is a controller, 
      // or if the device is named 'Wireless Controller'
      // If so, foundController is asserted.
      if (addressIsController(&pDevice->getAddress().toString().c_str()[0])
        || strcmp(pDevice->getName().c_str(), "Wireless Controller") == 0)
        foundController = true;
    } 
  );
}

/// @brief Stores the paired controller's MAC address into ESP32 `Preferences` (persistent memory)
/// @param addr the address (as a string-like) to store
/// @param clear whether to clear the preferences before storing. defaults to false.
void storeAddress(const char* addr, bool clear = false) {
  if (clear) prefs.clear();
  prefs.begin(PREF_KEY, false); // false means read/write mode

  // create 'String' from char array to store in preferences
  //* this is not std::string, it's an ESP thing
  String str(addr); 

  // Store MAC Address
  size_t size = prefs.putString(PREF_KEY, str);
  Serial.print(F("Storing MAC Address: "));
  Serial.print(&str.c_str()[0]);
  Serial.print(F(", of size "));
  Serial.println(size);
  prefs.end();
}

/// @brief Retrieves the stored controller MAC address from ESP32 `Preferences` (persistent memory)
/// @param addr the variable to place the address in. a more primitive version of a string, essentially.
/// Used: https://stackoverflow.com/questions/5660527/how-do-i-return-a-char-array-from-a-function
void getAddress(const char* &addr) {
  prefs.begin(PREF_KEY, true); // true is read-only mode
  String str = prefs.getString(PREF_KEY, "");
  Serial.print(F("Retrieved MAC Address: "));
  Serial.println(&str.c_str()[0]);
  prefs.end();
  if (str == "") addr = nullptr;
  else addr = &str.c_str()[0]; // get value of char ptr string
}

/// @brief Search for PS5 Controllers and pair to the first one found
/// @param doRePair whether or not to search for the controller whose MAC address is stored in non-volatile memory, default true
/// @param discoverTime the time limit to repair to existing devices, or search for new devices, in milliseconds
void activatePairing(bool doRePair, int discoverTime) {
  Serial.begin(115200);
  // pinMode(LED_BUILTIN, OUTPUT);

  // if we just returned a char*, it would be deleted and point to nowhere useful
  // so we have to pass in and mutate a (reference to a) char array.
  const char* addrCharPtr = nullptr;
  getAddress(addrCharPtr); 

  if (doRePair) {
    // see if we have a stored MAC address and try to pair to it
    if (addrCharPtr != nullptr) {
      Serial.print(F("Connecting to PS5 Controller @ "));
      Serial.println(addrCharPtr);
      ps5.begin(addrCharPtr);
      int timer = 0;

      // wait until discovery time passes or we connect to a controller
      while (timer < discoverTime && !ps5.isConnected()) {
        delay(LOOP_DELAY);
        timer += LOOP_DELAY;

        // slow blink when searching for previous device
        if (timer % (5 * LOOP_DELAY) == 0) {
          toggleBuiltInLED();
        }
      }

      // return if we get a connection at this point
      if (ps5.isConnected()) {
        Serial.println(F("PS5 Controller Connected!"));
        yeet;
      } // otherwise look for devices to pair with
    } 
  }

  // begin broadcasting as "ESP32" as master role
  if (!SerialBT.begin("ESP32", true)) { 
    Serial.println(F("SerialBT failed!")); // function returns false if failed
    abort();
  }
  SerialBT.enableSSP(); // according to SRC of this code, doesn't seem to change anything

  Serial.println(F("Searching for devices..."));
  BTScanResults* btDeviceList = SerialBT.getScanResults();  // may be accessing from different threads!

  // Beginning of Asynchronous Discovery Process
  if (startDiscovery()) {
    int timer = 0;

    // recall foundController is set by the callback in `startDiscovery` when a valid PS5 controller is found
    while (timer < discoverTime && !foundController) { 
      delay(LOOP_DELAY);
      timer += LOOP_DELAY;
      Lights::getInstance().updateLEDS();
      
      // double blink when in pairing mode like PS5 controller
      // at: 300/400, 600/700
      if ((timer % 1000) % (7 * LOOP_DELAY) == 0)
        toggleBuiltInLED();
      else if ((timer % 1000) % (4 * LOOP_DELAY) == 0)
        toggleBuiltInLED();
      else if ((timer % 1000) % (3 * LOOP_DELAY) == 0 &&
               (timer % 1000) % (9 * LOOP_DELAY) != 0) // also does 600
        toggleBuiltInLED();
    }

    Serial.println(F("Stopping discoverAsync... "));
    SerialBT.discoverAsyncStop();
    Serial.println(F("discoverAsync stopped"));
    delay(5000); //! why is this delay here? does removing it affect anything? this was in the original code, I must never have noticed it.
    
    // If we find devices, list them and try to pair if it is a valid controller.
    if(btDeviceList->getCount() > 0) {
      BTAddress addr;
      int channel = 0;
      Serial.println(F("Found devices:"));
      for (int i = 0; i < btDeviceList->getCount(); i++) {
        BTAdvertisedDevice* device = btDeviceList->getDevice(i);
        addr = device->getAddress();
        auto name = device->getName().c_str(); // get name to print and check
        auto addrStr = addr.toString().c_str(); // std::string doesn't work with Serial.print for some reason
        // ps5.begin requires a const char*, so get memory address of string/char array
        addrCharPtr = &addr.toString().c_str()[0]; // declared at top of function

        // print out relevant controller details
        // reminder that we need to use flash strings whenever possible, so don't try to collapse this
        Serial.print(i);
        Serial.print(F(" | "));
        Serial.print(addr.toString().c_str());
        Serial.print(F(" | "));
        Serial.print(device->getName().c_str());
        Serial.print(F(" | "));
        Serial.println(device->getRSSI());

        if (addressIsController(addrCharPtr) || strcmp(device->getName().c_str(), "Wireless Controller") == 0) {
          Serial.print(F("Connecting to PS5 Controller @ "));
          Serial.println(addrCharPtr);
          ps5.begin(addrCharPtr);
          while (!ps5.isConnected()) {
            toggleBuiltInLED(); // fast blinking when hooked into a device but not yet connected
            delay(LOOP_DELAY);
            Lights::getInstance().updateLEDS();
          }
          Serial.print(F("PS5 Controller Connected: "));
          Serial.println(ps5.isConnected());
          storeAddress(&addr.toString().c_str()[0], true);
          setBuiltInLED(true); // solid blue light when fully paired
        }
      }

      // if not connected at this point, no valid controllers have been found
      if (!ps5.isConnected()) setBuiltInLED(false); // turn the led off
    } else {
      Serial.println(F("Found no pairable devices."));
      setBuiltInLED(false);
    }
  } else {
    Serial.println(F("Asynchronous discovery failed."));
    setBuiltInLED(false);
  }
}