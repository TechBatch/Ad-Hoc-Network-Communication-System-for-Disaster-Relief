#include <esp_now.h>
#include <WiFi.h>

uint8_t MU1_MAC[] = {0xC8, 0x2E, 0x18, 0xEF, 0xF2, 0x68};
uint8_t MU2_MAC[] = {0xC8, 0xF0, 0x9E, 0x52, 0xE4, 0x84};
uint8_t MU3_MAC[] = {0x30, 0xC6, 0xF7, 0x42, 0xED, 0x1C};

typedef struct struct_message {
  int obstacles[3];
  int direction;
} struct_message;

// Create a struct_message called myData
struct_message espnow_MU1;
struct_message espnow_MU2;
struct_message espnow_MU3;

esp_now_peer_info_t peerInfoMU1;
esp_now_peer_info_t peerInfoMU2;
esp_now_peer_info_t peerInfoMU3;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
 
void setup() {
  int obs;
  // Init Serial Monitor
  Serial.begin(115200);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfoMU1.peer_addr, MU1_MAC, 6);
  peerInfoMU1.channel = 0;  
  peerInfoMU1.encrypt = false;
  
  memcpy(peerInfoMU2.peer_addr, MU2_MAC, 6);
  peerInfoMU2.channel = 0;  
  peerInfoMU2.encrypt = false;
  
  memcpy(peerInfoMU3.peer_addr, MU3_MAC, 6);
  peerInfoMU3.channel = 0;  
  peerInfoMU3.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfoMU1) != ESP_OK){
    Serial.println("Failed to add peer 1");
    return;
  }
  if (esp_now_add_peer(&peerInfoMU2) != ESP_OK){
    Serial.println("Failed to add peer 2");
    return;
  }
  if (esp_now_add_peer(&peerInfoMU3) != ESP_OK){
    Serial.println("Failed to add peer 3");
    return;
  }
  Serial.println("Enter 6 integer values:");
  for (int i = 0; i < 6; ++i) {
    while (!Serial.available()) {} // Wait for input
    if(i < 3) 
    {
      obs = Serial.parseInt();
      espnow_MU1.obstacles[i] = obs;
      espnow_MU2.obstacles[i] = obs;
      espnow_MU3.obstacles[i] = obs;
      Serial.println(obs,DEC);
    }
    else if(i == 3)
    {
      espnow_MU1.direction = Serial.parseInt();
    }
    else if(i == 4)
    {
      espnow_MU2.direction = Serial.parseInt();
    }
    else if(i == 5)
    {
      espnow_MU3.direction = Serial.parseInt();
    }
      // Clear the serial input buffer
    while (Serial.available()) 
    {
      Serial.read(); // Read and discard the characters
    }
  }

}
 
void loop() {
  // Send message via ESP-NOW
  esp_err_t result1 = esp_now_send(MU1_MAC, (uint8_t *) &espnow_MU1, sizeof(espnow_MU1));
   
  if (result1 == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }

  esp_err_t result2 = esp_now_send(MU2_MAC, (uint8_t *) &espnow_MU2, sizeof(espnow_MU2));
   
  if (result2 == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }

  esp_err_t result3 = esp_now_send(MU3_MAC, (uint8_t *) &espnow_MU3, sizeof(espnow_MU3));
   
  if (result3 == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }

  delay(2000);
}