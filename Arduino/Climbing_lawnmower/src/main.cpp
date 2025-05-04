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
struct_message joystick;

//Enable all motors
#define enableMotors_pin 12 

//BL Motor
#define directionMotor1_pin 27 
#define speedMotor1_pin 14

//BR Motor
#define directionMotor2_pin 25
#define speedMotor2_pin 26

//FL and FR Motor
#define directionMotor3and4_pin 4
#define speedMotor3and4_pin 16

//Joystick
#define joystickX 33 //temp
#define joystickY 32 //temp

void awakeMotors();
void disableMotors();
void enableMotors();
double Pid(double error);
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);

bool On = false;

bool DirectionBR;
bool DirectionBL;
bool DirectionFLandFR;
float SpeedBR = 0.0;
float SpeedBL = 0.0;
float SpeedFLandFR = 0.0;

float maxSpeed = 255.0;  //av 255

unsigned char callibrationX = 118;
unsigned char callibrationY = 116;
unsigned char threshold = 100;
long t1;
long t2 = 1;
long deltaT_ME;
long previousTime = 0;
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
  pinMode(directionMotor3and4_pin, OUTPUT);
  pinMode(speedMotor1_pin, OUTPUT);
  pinMode(speedMotor2_pin, OUTPUT);
  pinMode(speedMotor3and4_pin, OUTPUT);

  pinMode(joystickX, INPUT);
  pinMode(joystickY, INPUT);

  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock.
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  Serial.begin(115200); //Serial communication
  delay(1000);
}

void loop()
{
  joystick.x = analogRead(joystickX)/4095.0*255.0;
  joystick.y = analogRead(joystickY)/4095.0*255.0;

  double now = micros();
  dt = (now - last_time)/1000.0;
  last_time = now;

  // pid = Pid(Roll);

 
  //Joystick mapping
  //state 1 - Up 
  if(callibrationX - threshold < joystick.x && joystick.x < callibrationX + threshold && joystick.y > callibrationY + threshold)
  {
    Serial.print("1, ");
    SpeedBL = 1.0*255;
    SpeedBR = 1.0*255;
    DirectionBL = false;
    DirectionBR = true;

    SpeedFLandFR = joystick.y-callibrationY;
    DirectionFLandFR = true;
  } 
  //state 2 - Diagonally up-right
  else if(joystick.x > callibrationX + threshold && joystick.y > callibrationY + threshold)
  {
    Serial.print("2, ");
    SpeedBL = 0.5*255;
    SpeedBR = 0.5*255;
    DirectionBL = false;
    DirectionBR = true;

    SpeedFLandFR = 0.0*255;
  }
  //state 3 - Right
  else if(joystick.x > callibrationX  + threshold && callibrationY - threshold < joystick.y && joystick.y < callibrationY + threshold)
  {
    Serial.print("3, ");
    SpeedBL = 0.75*255;
    SpeedBR = 0.75*255;
    DirectionBL = false;
    DirectionBR = false; 

    SpeedFLandFR = 0.0*255;
  }
  //state 4 - Diagonally down-right
  else if(joystick.x > callibrationX + threshold && joystick.y < callibrationY - threshold)
  {
    Serial.print("4, ");
    SpeedBL = 0.5*255;
    SpeedBR = 1.0*255;
    DirectionBL = true;
    DirectionBR = false;

    SpeedFLandFR = 0.0*255;
  }
  //state 5 - Down
  else if(callibrationX - threshold < joystick.x && joystick.x < callibrationX + threshold && joystick.y < callibrationY - threshold)
  {
    Serial.print("5, ");
    SpeedBL = 1.00*255;
    SpeedBR = 1.00*255;
    DirectionBL = true;
    DirectionBR = false; 

    SpeedFLandFR = 0.0*255;
  }
  //state 6 - Diagonally down-left
  else if(joystick.x < callibrationX - threshold && joystick.y < callibrationY - threshold)
  {
    Serial.print("6, ");
    SpeedBL = 1.00*255;
    SpeedBR = 1.00*255;
    DirectionBL = true;
    DirectionBR = false; 

    SpeedFLandFR = 0.0*255;
  }
  //state 7 - Left
  else if(joystick.x < callibrationX - threshold && callibrationY - threshold < joystick.y && joystick.y < callibrationY + threshold)
  {
    Serial.print("7, ");
    SpeedBL = 0.75*255;
    SpeedBR = 0.75*255;
    DirectionBL = true;
    DirectionBR = true; 

    SpeedFLandFR = 0.0*255;
  }
  //state 8 - Diagonally up-left
  else if(joystick.x < callibrationX - threshold && joystick.y > callibrationY + threshold)
  {
    Serial.print("8, ");
    SpeedBL = 0.50*255;
    SpeedBR = 1.00*255;
    DirectionBL = false;
    DirectionBR = true; 

    SpeedFLandFR = 0.0*255;
  }
  //state 9 - Idle
  else
  {
    Serial.print("9, ");
    SpeedBL = 0.0*255;
    SpeedBR = 0.0*255;
    DirectionBL = false;
    DirectionBR = true;
    
    SpeedFLandFR = 0.0*255;
  }

  awakeMotors();
  
  if (SpeedBR >= maxSpeed)
    SpeedBR = maxSpeed;
  
  if (SpeedBL >= maxSpeed)
    SpeedBL = maxSpeed;
  //if (Speed <= 0)
  //  Speed = 0;
  analogWrite(speedMotor1_pin, SpeedBL);
  analogWrite(speedMotor2_pin, SpeedBR);
  digitalWrite(directionMotor1_pin, DirectionBR);
  digitalWrite(directionMotor2_pin, DirectionBL);
  analogWrite(speedMotor3and4_pin, SpeedFLandFR);
  digitalWrite(directionMotor3and4_pin, DirectionFLandFR);

#if (defined enableSerialprint)
  Serial.print("SpeedFLandFR: ");
  Serial.print(SpeedFLandFR);
  Serial.print(", SpeedBL: ");
  Serial.print(SpeedBL);
  Serial.print(", SpeedBR: ");
  Serial.print(SpeedBR);
  Serial.print(", myData.x: ");
  Serial.print(joystick.x);
  Serial.print(", myData.y: ");
  Serial.print(joystick.y);
  Serial.print(", DirectionBL: ");
  Serial.print(DirectionBL);
  Serial.print(", DirectionBR: ");
  Serial.println(DirectionBR);

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
void awakeMotors()
{
  if(millis() > previousTime + 3000)
  {
    disableMotors();
    previousTime = millis();
  }
  else
    enableMotors();
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
  delayMicroseconds(1);
}