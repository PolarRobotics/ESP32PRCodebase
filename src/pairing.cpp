#include <Arduino.h>
#ifdef ESP32
  #include <WiFi.h>
#else
  #include <ESP8266WiFi.h>
#endif

/*
   Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleScan.cpp
   Ported to Arduino ESP32 by Evandro Copercini
*/

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

int scanTime = 5; //In seconds
BLEScan* pBLEScan;

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      Serial.printf("Advertised Device: %s \n", advertisedDevice.toString().c_str());
    }
};

void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200);
    Serial.println();
    Serial.print(F("ESP Board MAC Address:  "));
    Serial.println(WiFi.macAddress());
    // 30:C6:F7:29:ED:24 for the ESP32 I was testing

    Serial.println("Scanning...");

    BLEDevice::init("");
    pBLEScan = BLEDevice::getScan(); //create new scan
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster
    pBLEScan->setInterval(100);
    pBLEScan->setWindow(99);  // less or equal setInterval value
    BLEScanResults foundDevices = pBLEScan->start(scanTime, false);
    Serial.print("Devices found: ");
    Serial.println(foundDevices.getCount());
    for (int i = 0; i < foundDevices.getCount(); i++) {
        if (foundDevices.getDevice(i).isAdvertisingService(foundDevices.getDevice(i).getServiceUUID())) {
            Serial.print("Device advertising service: ");
            Serial.print(i);
            Serial.print(", UUID: ");
            Serial.println(foundDevices.getDevice(i).getServiceUUID().toString().c_str());
        }
    }
    Serial.println("Scan done!");
    pBLEScan->clearResults();   // delete results fromBLEScan buffer to release memory
    delay(2000);
}

void loop() {

}