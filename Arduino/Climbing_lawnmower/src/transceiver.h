#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

#define upPin 25
#define downPin 23
#define leftPin 22
#define rightPin 14

#define pistonF_Up_Pin 27
#define pistonF_Down_Pin 26
#define pistonB_Up_Pin 17
#define pistonB_Down_Pin 16

uint8_t broadcastAddress[] = {0x24, 0xDC, 0xC3, 0x9F, 0xE1, 0xA0};
typedef struct struct_message {
  //Movment
  bool up;
  bool down;
  bool left;
  bool right;
  //Pistons
  bool pistonF_Up;
  bool pistonF_Down;
  bool pistonB_Up;
  bool pistonB_Down;
} struct_message;
struct_message transmitterData;
esp_now_peer_info_t peerInfo;
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  //Movment
  pinMode(upPin, INPUT_PULLDOWN);
  pinMode(downPin, INPUT_PULLDOWN);
  pinMode(leftPin, INPUT_PULLDOWN);
  pinMode(rightPin, INPUT_PULLDOWN);
  //Pistons
  pinMode(pistonF_Up_Pin, INPUT_PULLDOWN);
  pinMode(pistonF_Down_Pin, INPUT_PULLDOWN);
  pinMode(pistonB_Up_Pin, INPUT_PULLDOWN);
  pinMode(pistonB_Down_Pin, INPUT_PULLDOWN);

  if (esp_now_init() != ESP_OK) {return;}
  esp_now_register_send_cb(OnDataSent);
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false; 
  if (esp_now_add_peer(&peerInfo) != ESP_OK){return;}
}

void loop() {
  transmitterData.up = digitalRead(upPin);
  transmitterData.down = digitalRead(downPin);
  transmitterData.left = digitalRead(leftPin);
  transmitterData.right = digitalRead(rightPin);

  transmitterData.pistonF_Up = digitalRead(pistonF_Up_Pin);
  transmitterData.pistonF_Down = digitalRead(pistonF_Down_Pin);
  transmitterData.pistonB_Up = digitalRead(pistonB_Up_Pin);
  transmitterData.pistonB_Down = digitalRead(pistonB_Down_Pin);
  //Movement
  Serial.print("Up: ");
  Serial.print(transmitterData.up);
  Serial.print(", Down: ");
  Serial.print(transmitterData.down);
  Serial.print(", Left: ");
  Serial.print(transmitterData.left);
  Serial.print(", Right: ");
  Serial.print(transmitterData.right);
  //Pistons
  Serial.print(", PistonF_Up: ");
  Serial.print(transmitterData.pistonF_Up);
  Serial.print(", PistonF_Down: ");
  Serial.print(transmitterData.pistonF_Down);
  Serial.print(", PistonB_Up: ");
  Serial.print(transmitterData.pistonB_Up);
  Serial.print(", PistonB_Down: ");
  Serial.println(transmitterData.pistonB_Down);

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &transmitterData, sizeof(transmitterData));
  delay(10);
}