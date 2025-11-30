/*
 * ESP8266 Receiver Code using ESP-NOW (Continuous Mode)
 * 
 * Functionality:
 * - Receives continuous button state updates from transmitter
 * - Controls 4 digital outputs with inverted logic
 * - Default state: HIGH (pulled up)
 * - When button pressed: Output goes LOW (pulled to ground)
 * - Perfect for RC applications with continuous updates
 */

#include <ESP8266WiFi.h>
#include <espnow.h>

// GPIO pin assignments for ESP8266 NodeMCU/Wemos D1 Mini
// Using D pins nomenclature
#define OUTPUT1_PIN D1   // GPIO14 for output 1
#define OUTPUT2_PIN D0   // GPIO12 for output 2
#define OUTPUT3_PIN D5   // GPIO13 for output 3
#define OUTPUT4_PIN D2   // GPIO15 for output 4

// Output state tracking
bool output1State = false;
bool output2State = false;
bool output3State = false;
bool output4State = false;

// Structure to receive data
// Must match the transmitter structure
typedef struct message {
  bool button1;
  bool button2;
  bool button3;
  bool button4;
} message;

// Create a structured object
message myData;

// Callback function executed when data is received
void OnDataRecv(uint8_t *mac, uint8_t *incomingData, uint8_t len) {
  memcpy(&myData, incomingData, sizeof(myData));
  
  // INVERTED LOGIC:
  // Button pressed (true) = Output LOW (pulled to ground)
  // Button released (false) = Output HIGH (pulled up)
  
  // Update all outputs based on received button states
  digitalWrite(OUTPUT1_PIN, !myData.button1);
  digitalWrite(OUTPUT2_PIN, !myData.button2);
  digitalWrite(OUTPUT3_PIN, !myData.button3);
  digitalWrite(OUTPUT4_PIN, !myData.button4);
  
  // Optional: Debug output (uncomment if needed)
  // if (Serial) {
  //   Serial.print("Received: B1=");
  //   Serial.print(myData.button1 ? "ON" : "OFF");
  //   Serial.print(" B2=");
  //   Serial.print(myData.button2 ? "ON" : "OFF");
  //   Serial.print(" B3=");
  //   Serial.print(myData.button3 ? "ON" : "OFF");
  //   Serial.print(" B4=");
  //   Serial.println(myData.button4 ? "ON" : "OFF");
  // }
}

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // Initialize outputs as digital outputs
  pinMode(OUTPUT1_PIN, OUTPUT);
  pinMode(OUTPUT2_PIN, OUTPUT);
  pinMode(OUTPUT3_PIN, OUTPUT);
  pinMode(OUTPUT4_PIN, OUTPUT);
  
  // Set initial state to HIGH (pulled up)
  digitalWrite(OUTPUT1_PIN, HIGH);
  digitalWrite(OUTPUT2_PIN, HIGH);
  digitalWrite(OUTPUT3_PIN, HIGH);
  digitalWrite(OUTPUT4_PIN, HIGH);

  // Initialize ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Set ESP-NOW role
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  
  // Register callback function
  esp_now_register_recv_cb(OnDataRecv);
  
  Serial.println("Receiver initialized");
  
  // Print this device's MAC address for configuration of the transmitter
  Serial.print("This device MAC address: ");
  Serial.println(WiFi.macAddress());
}

void loop() {
  // The receiver mainly relies on the callback function
  // You can add additional logic here if needed
  
  // Small delay for stability
  delay(10);
}