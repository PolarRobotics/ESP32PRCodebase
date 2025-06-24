// QbBaseComms.cpp
// This file is meant to be uploaded to the second ESP32 that is mounted in the QB Base.
// It is used to communicate with the QB turret comms ESP32 using ESP-NOW, while also receiving data from the QB base via UART.

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

// ! This is the MAC of the ESP on the turret. If you change the ESP on the turret, you must change this address to match the new ESP's MAC address.
const uint8_t QB_TURRET_ADDR[6] = {0xF8, 0xB3, 0xB7, 0x46, 0xBA, 0xC8}; // MAC address of the QB turret

#define NUM_CHARS 100

void recvWithStartEndMarkers();
void parseData();

char receivedChars[NUM_CHARS];
char tempChars[NUM_CHARS];
bool newData = false;
int16_t targetRelativeHeading = 0; // Target relative heading for the turret, recieived from the QB base

// Define the message structure to send to the QB turret
typedef struct message {
  int16_t heading; // Target relative heading for the turret, to send to the QB turret
} message;


message data;

// Function prototype for the ESP-NOW send callback
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);

HardwareSerial UART_BASE = Serial2; // Use Serial2 for UART communication with the QB base

void setup() {
  Serial.begin(115200);
  UART_BASE.begin(115200, SERIAL_8N1, 16, 17); // RX on GPIO 16, TX on GPIO 17

  // Setup ESP-NOW
  WiFi.mode(WIFI_STA);
  if(esp_now_init() != ESP_OK){
    Serial.println("Error initializing");
    return;
  }
  esp_now_register_send_cb(onDataSent);
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, QB_TURRET_ADDR, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA;
  if(esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

}

void loop() {
  esp_err_t err = 1;

  // Read data from QB Base main via UART
  // This data is expected to be in the format <targetRelativeHeading>
  recvWithStartEndMarkers();
  if(newData == true){
    strcpy(tempChars,receivedChars);
    parseData();
    Serial.print("Target Relative Heading: ");
    Serial.println(targetRelativeHeading);
    newData = false;
  }
  // Send the data to the QB turret via ESP-NOW
  while(err != ESP_OK){
    err = esp_now_send(QB_TURRET_ADDR, (uint8_t*) &data, sizeof(data));
    Serial.println(esp_err_to_name(err));
  }
  delay(20);
}

// Callback function to handle the status of the sent data
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status){
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery failed");
}

// Function to receive data from UART with start and end markers
void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;
    
    while (UART_BASE.available() > 0 && newData == false) {
        // Serial.println("Data Received");
        rc = UART_BASE.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= NUM_CHARS) {
                    ndx = NUM_CHARS - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }
        
        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

// Function to parse the received data and extract the target relative heading
void parseData() {      // split the data into its parts
    char * strtokIndx; // this is used by strtok() as an index

    strtokIndx = strtok(tempChars,",");
    if(strtokIndx != NULL){
        targetRelativeHeading = atoi(strtokIndx);
    }
    data.heading = targetRelativeHeading; // Update the message struct with the target relative heading

}