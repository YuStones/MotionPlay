#include <esp_now.h>
#include <WiFi.h>

typedef struct struct_message {
  int quatW;
  int quatX;
  int quatY;
  int quatZ;
} struct_message;

struct_message data;

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&data, incomingData, sizeof(data));
  Serial.println(data.quatW);
  Serial.println(data.quatX);
  Serial.println(data.quatY);
  Serial.println(data.quatZ);
}

void setup() {
  Serial.begin(250000);
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);

}

void loop() {

}