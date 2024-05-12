#include <esp_now.h>
#include <WiFi.h>
int WiFiFlag = 0;
// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
    int obstacle[3];
    int direction;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Obstacle: ");
  Serial.println(myData.obstacle[0]);
  Serial.print("Obstacle: ");
  Serial.println(myData.obstacle[1]);
  Serial.print("Obstacle: ");
  Serial.println(myData.obstacle[2]);
  Serial.print("Direction: ");
  Serial.println(myData.direction);
  Serial.println();
  WiFiFlag = 1;
}

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
  while(!WiFiFlag){}
  esp_now_unregister_recv_cb();
}
 
void loop() {

}