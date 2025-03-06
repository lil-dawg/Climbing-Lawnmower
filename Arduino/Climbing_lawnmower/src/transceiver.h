#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

#define potx 32
#define poty 33
uint8_t broadcastAddress[] = {0x24, 0xDC, 0xC3, 0x9F, 0xE1, 0xA0};
typedef struct struct_message {
  unsigned char potValx;
  unsigned char potValy;
} struct_message;
struct_message myData;
esp_now_peer_info_t peerInfo;
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  pinMode(potx, INPUT_PULLUP);
  pinMode(poty, INPUT_PULLUP);

  if (esp_now_init() != ESP_OK) {return;}
  esp_now_register_send_cb(OnDataSent);
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false; 
  if (esp_now_add_peer(&peerInfo) != ESP_OK){return;}
}

void loop() {
  myData.potValx = analogRead(potx)/4095.0*255.0;
  Serial.print("X: ");
  Serial.print(myData.potValx);
  myData.potValy = analogRead(poty)/4095.0*255.0;
  Serial.print(", Y: ");
  Serial.println(myData.potValy);
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
  delay(50);
}
