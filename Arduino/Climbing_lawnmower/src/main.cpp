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

#define INTERRUPT_PIN 2  
#define EnableMotors_pin 12 
#define DirectionMotor1_pin 27 
#define DirectionMotor2_pin 32 
#define pwmSignalMotor1_pin 13 
#define pwmSignalMotor2_pin 33 

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

  pinMode(EnableMotors_pin   , OUTPUT);
  pinMode(DirectionMotor1_pin, OUTPUT);
  pinMode(DirectionMotor2_pin, OUTPUT);
  pinMode(pwmSignalMotor1_pin, OUTPUT);
  pinMode(pwmSignalMotor2_pin, OUTPUT);

  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(115200);
  pinMode(INTERRUPT_PIN, INPUT);
}

void loop()
{
  double now = micros();
  dt = (now - last_time)/1000.0;
  last_time = now;

  // pid = Pid(Roll);
  
  SpeedR = sqrt(2*pow(myData.x - 113,2) + 2*pow(myData.y - 113,2));
  SpeedL = sqrt(2*pow(myData.x - 113,2) + 2*pow(myData.y - 113,2));

  if(myData.y - 113 >= 0)
  {
    DirectionR = true;
    DirectionL = false;
  }
    
  else if(myData.y - 113 < -0)
  {
    DirectionR = false;
    DirectionL = true;
  }
    
  else if(myData.x - 113 >= 0)
  {
    DirectionR = true;
    DirectionL = true;
  }
    
  else if(myData.x - 113 < -0)
  {
    DirectionR = false;
    DirectionL = false;
  }

  enableMotors();

  if (SpeedR >= maxSpeed)
    SpeedR = maxSpeed;
  
  if (SpeedL >= maxSpeed)
    SpeedL = maxSpeed;
  //if (Speed <= 0)
  //  Speed = 0;
  analogWrite(pwmSignalMotor1_pin, SpeedL);
  analogWrite(pwmSignalMotor2_pin, SpeedR);
  digitalWrite(DirectionMotor1_pin, DirectionR);
  digitalWrite(DirectionMotor2_pin, DirectionL);

#if (defined enableSerialprint)
  Serial.print("SpeedL: ");
  Serial.print(SpeedL);
  Serial.print(", SpeedR: ");
  Serial.print(SpeedR);
  Serial.print(", myData.x: ");
  Serial.print(myData.x);
  Serial.print(", myData.y: ");
  Serial.print(myData.y);
  // Serial.print(", Pid: "); 
  // Serial.print(pid);
  // Serial.print(", Acc: ");
  // Serial.print(Acc);
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
  pinMode(EnableMotors_pin, INPUT);
  delayMicroseconds(10);
}
void disableMotors()
{
  pinMode(EnableMotors_pin, OUTPUT);
  digitalWrite(EnableMotors_pin, LOW);
  delayMicroseconds(10);
}
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
