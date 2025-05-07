#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

// #define potx 32
// #define poty 33

#define upPin 25
#define downPin 26
#define leftPin 27
#define rightPin 14

#define pistonF_Up_Pin 26
#define pistonF_Down_Pin 27
#define pistonB_Up_Pin 17
#define pistonB_Down_Pin 16

uint8_t broadcastAddress[] = {0x24, 0xDC, 0xC3, 0x9F, 0xE1, 0xA0};
typedef struct struct_message {
  unsigned char potValx;
  unsigned char potValy;
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
struct_message myData;
esp_now_peer_info_t peerInfo;
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  //Joystick
  // pinMode(potx, INPUT_PULLUP);
  // pinMode(poty, INPUT_PULLUP);

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
  // myData.potValx = analogRead(potx)/4095.0*255.0;
  // Serial.print("X: ");
  // Serial.print(myData.potValx);
  // myData.potValy = analogRead(poty)/4095.0*255.0;
  // Serial.print(", Y: ");
  // Serial.println(myData.potValy);

  myData.up = digitalRead(upPin);
  myData.down = digitalRead(downPin);
  myData.left = digitalRead(leftPin);
  myData.right = digitalRead(rightPin);

  myData.pistonF_Up = digitalRead(pistonF_Up_Pin);
  myData.pistonF_Down = digitalRead(pistonF_Down_Pin);
  myData.pistonB_Up = digitalRead(pistonB_Up_Pin);
  myData.pistonB_Down = digitalRead(pistonB_Down_Pin);

  Serial.print("Up: ");
  Serial.print(myData.up);
  Serial.print(", Down: ");
  Serial.print(myData.down);
  Serial.print(", Left: ");
  Serial.print(myData.left);
  Serial.print(", Right: ");
  Serial.println(myData.right);

  Serial.print("PistonF_Up: ");
  myData.pistonF_Up = digitalRead(pistonF_Up_Pin);
  Serial.print(myData.pistonF_Up);
  Serial.print(", PistonF_Down: ");
  myData.pistonF_Down = digitalRead(pistonF_Down_Pin);
  Serial.print(myData.pistonF_Down);
  Serial.print(", PistonB_Up: ");
  myData.pistonB_Up = digitalRead(pistonB_Up_Pin);
  Serial.print(myData.pistonB_Up);
  Serial.print(", PistonB_Down: ");
  myData.pistonB_Down = digitalRead(pistonB_Down_Pin);
  Serial.println(myData.pistonB_Down);

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
  delay(10);
}