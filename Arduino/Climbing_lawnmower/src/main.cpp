#include <Arduino.h>
#include "I2Cdev.h"
#include "Wire.h"
#include "PID_v1.h"
#include <esp_now.h>
#include <WiFi.h>

#define enableSerialprint

typedef struct struct_message {
  unsigned char x;
  unsigned char y;
} struct_message;

struct_message myData;
struct_message joystick;

#define enableMotors_pin 12 

#define directionMotor1_pin 27 
#define speedMotor1_pin 14

#define directionMotor2_pin 25
#define speedMotor2_pin 26 

#define directionMotor3and4_pin null
#define speedMotor3and4_pin null

#define joystickX 33 //temp
#define joystickY 32 //temp

void disableMotors();
void enableMotors();
double Pid(double error);
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max);
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);

bool On = false;
bool DirectionR;
bool DirectionL;
float SpeedR = 0.0;
float SpeedL = 0.0;
float Acc = 0.0;
float maxSpeed = 255.0;  //av 255
unsigned char callibrationX = 118;
unsigned char callibrationY = 116;
unsigned char threshold = 100;
long t1;
long t2 = 1;
long deltaT_ME;
//PID
double dt, last_time;
double integral, previous, pid = 0;
double kp = 1;
double ki = 0.0; //skippa helt
double kd = 5; //20

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  // Serial.print("x: ");
  // Serial.println(myData.x);
  // Serial.print("y: ");
  // Serial.println(myData.y);
}

void setup()
{
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {return;}
  esp_now_register_recv_cb(OnDataRecv);

  pinMode(enableMotors_pin   , OUTPUT);
  pinMode(directionMotor1_pin, OUTPUT);
  pinMode(directionMotor2_pin, OUTPUT);
  pinMode(speedMotor1_pin, OUTPUT);
  pinMode(speedMotor2_pin, OUTPUT);

  pinMode(joystickX, INPUT);
  pinMode(joystickY, INPUT);

  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(115200);
}

void loop()
{
  joystick.x = analogRead(joystickX)/4095.0*255.0;
  joystick.y = analogRead(joystickY)/4095.0*255.0;

  double now = micros();
  dt = (now - last_time)/1000.0;
  last_time = now;

  // pid = Pid(Roll);

  if(callibrationX - threshold < joystick.x && joystick.x < callibrationX + threshold && joystick.y > callibrationY + threshold)
  {
    Serial.print("1, ");
    SpeedL = 1.0*255;
    SpeedR = 1.0*255;
    DirectionL = true;
    DirectionR = false;
  } //1
  else if(joystick.x > callibrationX + threshold && joystick.y > callibrationY + threshold)
  {
    Serial.print("2, ");
    SpeedL = 0.5*255;
    SpeedR = 0.5*255;
    DirectionL = true;
    DirectionR = false;
  } //2
  else if(joystick.x > callibrationX  + threshold && callibrationY - threshold < joystick.y && joystick.y < callibrationY + threshold)
  {
    Serial.print("3, ");
    SpeedL = 0.75*255;
    SpeedR = 0.75*255;
    DirectionL = true;
    DirectionR = true; 
  } //3
  else if(joystick.x > callibrationX + threshold && joystick.y < callibrationY - threshold)
  {
    Serial.print("4, ");
    SpeedL = 0.5*255;
    SpeedR = 1.0*255;
    DirectionL = false;
    DirectionR = true;
  } //4
  else if(callibrationX - threshold < joystick.x && joystick.x < callibrationX + threshold && joystick.y < callibrationY - threshold)
  {
    Serial.print("5, ");
    SpeedL = 1.00*255;
    SpeedR = 1.00*255;
    DirectionL = false;
    DirectionR = true; 
  } //5
  else if(joystick.x < callibrationX - threshold && joystick.y < callibrationY - threshold)
  {
    Serial.print("6, ");
    SpeedL = 1.00*255;
    SpeedR = 1.00*255;
    DirectionL = false;
    DirectionR = true; 
  } //6
  else if(joystick.x < callibrationX - threshold && callibrationY - threshold < joystick.y && joystick.y < callibrationY + threshold)
  {
    Serial.print("7, ");
    SpeedL = 0.75*255;
    SpeedR = 0.75*255;
    DirectionL = false;
    DirectionR = false; 
  } //7
  else if(joystick.x < callibrationX - threshold && joystick.y > callibrationY + threshold)
  {
    Serial.print("8, ");
    SpeedL = 0.50*255;
    SpeedR = 1.00*255;
    DirectionL = true;
    DirectionR = false; 
  } //8
  else
  {
    Serial.print("9, ");
    SpeedL = 0.0*255;
    SpeedR = 0.0*255;
    DirectionL = true;
    DirectionR = false; 
  }

  enableMotors();

  if (SpeedR >= maxSpeed)
    SpeedR = maxSpeed;
  
  if (SpeedL >= maxSpeed)
    SpeedL = maxSpeed;
  //if (Speed <= 0)
  //  Speed = 0;
  analogWrite(speedMotor1_pin, SpeedL);
  analogWrite(speedMotor2_pin, SpeedR);
  digitalWrite(directionMotor1_pin, DirectionR);
  digitalWrite(directionMotor2_pin, DirectionL);

#if (defined enableSerialprint)
  Serial.print("SpeedL: ");
  Serial.print(SpeedL);
  Serial.print(", SpeedR: ");
  Serial.print(SpeedR);
  Serial.print(", myData.x: ");
  Serial.print(joystick.x);
  Serial.print(", myData.y: ");
  Serial.print(joystick.y);
  // // Serial.print(", Pid: "); 
  // // Serial.print(pid);
  // // Serial.print(", Acc: ");
  // // Serial.print(Acc);
  Serial.print(", DirectionL: ");
  Serial.print(DirectionL);
  Serial.print(", DirectionR: ");
  Serial.println(DirectionR);

#endif

}


double Pid(double error)
{
  double proportional = error;
  integral += error * dt;
  double derivative = (error - previous) / dt;
  previous = error;
  double output = (kp * proportional) + (ki * integral) + (kd * derivative);
  return output;
}
void enableMotors()
{
  pinMode(enableMotors_pin, INPUT);
  delayMicroseconds(10);
}
void disableMotors()
{
  pinMode(enableMotors_pin, OUTPUT);
  digitalWrite(enableMotors_pin, LOW);
  delayMicroseconds(10);
}
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
