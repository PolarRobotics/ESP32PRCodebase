// QbTurretComms.cpp
// This file is meant to be uploaded to the ESP32 that is mounted on the QB turret.
// It is used to communicate with the QB base via ESP-NOW, while also sending data to the QB Turret via UART.

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

// ! This is the MAC of the ESP on the QB base. If you change the ESP on the base, you must change this address to match the new ESP's MAC address.
const uint8_t QB_BASE_ADDR[6] = {0xF8, 0xB3, 0xB7, 0x3F, 0x33, 0xEC}; // MAC address of the QB base

typedef struct message {
  int16_t targetRelativeHeading; // Target relative heading for the turret, to send to the QB turret
} message;

message incomingMessage; // Message structure to receive from the QB base

void onDataReceive(const uint8_t *mac_addr, const uint8_t *incomingData, int len);

#define NUM_CHARS 40

void sendData(int x); // Send data to the turret main over uart.

char data[NUM_CHARS];

void setup() {
  Serial.begin(115200);

  // Setup ESP-NOW
  WiFi.mode(WIFI_STA);
  if(esp_now_init() != ESP_OK){
    Serial.println("Error initializing");
    return;
  }
  esp_now_register_recv_cb(onDataReceive); // Register the callback function to handle incoming data
}

void loop() {
    // Nothing to do here, all handled in onDataReceive
}

/**
 * @brief Callback function to handle incoming data from the QB base via ESP-NOW.
 * When data is received over ESP-NOW, this function is called.
 * It takes the incoming data, and sends it to the QB turret via UART.
 */
void onDataReceive(const uint8_t *mac_addr, const uint8_t *incomingData, int len){
    memcpy(&incomingMessage, incomingData, sizeof(message));
    sendData(incomingMessage.targetRelativeHeading); // Send the target relative heading to the QB base
    Serial.print("Target Heading: ");
    Serial.println(incomingMessage.targetRelativeHeading);
}

/**
 * @brief Function to send data to the QB turret via UART.
 * It formats the data as a string and sends it wrapped in angle brackets.
 */
void sendData(int x){
    snprintf(data, NUM_CHARS, "%d", x);
    Serial2.print("<");
    Serial2.print(data);
    Serial2.print(">");
}