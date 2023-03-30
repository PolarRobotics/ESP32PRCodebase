#include <Arduino.h>
#include "esp_bt_main.h"
#include "esp_bt_device.h"

// this file gets the MAC address of the esp, which isn't really that useful

bool initBluetooth()
{
  if (!btStart()) {
    Serial.println("Failed to initialize controller");
    return false;
  }
 
  if (esp_bluedroid_init() != ESP_OK) {
    Serial.println("Failed to initialize bluedroid");
    return false;
  }
 
  if (esp_bluedroid_enable() != ESP_OK) {
    Serial.println("Failed to enable bluedroid");
    return false;
  }
  return true;
}
 
void printDeviceAddress() {
 
  const uint8_t* point = esp_bt_dev_get_address();
 
  for (int i = 0; i < 6; i++) {
 
    char str[3];
 
    sprintf(str, "%02X", (int)point[i]);
    Serial.print(str);
 
    if (i < 5){
      Serial.print(":");
    }
 
  }
}

void setup() {
    initBluetooth();
    printDeviceAddress(); // FOUND E0:5A:1B:77:20:26 with the first ESP I tested 2023-02-16
}

void loop() {}