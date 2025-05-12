#include <Arduino.h>
#include "I2Cdev.h"
#include "Wire.h"
#include "PID_v1.h"
#include <esp_now.h>
#include <WiFi.h>

#define enableSerialprint

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
struct_message buttons;

//Piston DC motor 
  //PistonF
  #define cwDcFPin 19
  #define ccwDcFPin 18
  //PistonB
  #define cwDcBPin 32
  #define ccwDcBPin 13

//BLDC-motors
  //Enable all BLDC-motors
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

  #define enableStepperMotor_pin 33

void awakeMotors();
void disableMotors();
void enableMotors();
double Pid(double error);
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);

bool DirectionBR;
bool DirectionBL;
bool DirectionFLandFR;
float SpeedBR = 0.0;
float SpeedBL = 0.0;
float SpeedFLandFR = 0.0;

float maxSpeed = 255.0;  //av 255

long previousTime = 0;

//PID
double dt, last_time;
double integral, previous, pid = 0;
double kp = 1;
double ki = 0.0; //skippa helt
double kd = 5; //20
long t1;
long t2;
long deltaT_ME;

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&buttons, incomingData, sizeof(buttons));
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
  pinMode(speedMotor3and4_pin, OUTPUT);

  pinMode(enableStepperMotor_pin, OUTPUT);

  pinMode(cwDcFPin, OUTPUT);
  pinMode(ccwDcFPin, OUTPUT);
  pinMode(cwDcBPin, OUTPUT);
  pinMode(ccwDcBPin, OUTPUT);

  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock.
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  Serial.begin(115200); //Serial communication
}

void loop()
{
  double now = micros();
  dt = (now - last_time)/1000.0;
  last_time = now;

  // pid = Pid(Roll);

  //Movment ------------------------------------------------------------------------------
  if(buttons.up)
  {
    Serial.print("Up, ");
    SpeedBL = 0.5*255;
    SpeedBR = 0.5*255;
    DirectionBL = false;
    DirectionBR = true;

    SpeedFLandFR = 0.5*255;
    DirectionFLandFR = true;
  }
  else if(buttons.down)
  {
    Serial.print("Down, ");
    SpeedBL = 0.5*255;
    SpeedBR = 0.5*255;
    DirectionBL = true;
    DirectionBR = false; 

    SpeedFLandFR = 0.5*255;
    DirectionFLandFR = false;
  }
  else if(buttons.left)
  {
    Serial.print("Left, ");
    SpeedBL = 0.5*255;
    SpeedBR = 0.5*255;
    DirectionBL = false;
    DirectionBR = false; 
    
    SpeedFLandFR = 0.0*255;
    DirectionFLandFR = true;
  }
  else if(buttons.right)
  {
    Serial.println("Right, ");
    SpeedBL = 0.5*255;
    SpeedBR = 0.5*255;
    DirectionBL = true;
    DirectionBR = true; 
    
    SpeedFLandFR = 0.0*255;
    DirectionFLandFR = true;
  }
  else
  {
    Serial.print("Idle, ");
    SpeedBL = 0.0*255;
    SpeedBR = 0.0*255;
    DirectionBL = false;
    DirectionBR = true;

    SpeedFLandFR = 0.0*255;
    DirectionFLandFR = true;
  }

  awakeMotors(); //to make sure the drivers arn't going to sleep mode
  
  if (SpeedBR >= maxSpeed)
    SpeedBR = maxSpeed;
  
  if (SpeedBL >= maxSpeed)
    SpeedBL = maxSpeed;

//Pistons -----------------------------------------------------------------
    //PistonF
      //PistonF_Down
      if (buttons.pistonF_Down) {
        digitalWrite(cwDcFPin, LOW);
        digitalWrite(ccwDcFPin, HIGH);
      }
      //PistonF_Up
      else if (buttons.pistonF_Up) {
        digitalWrite(cwDcFPin, HIGH);
        digitalWrite(ccwDcFPin, LOW);
      }
    //PistonB
      //PistonB_Down
      if(buttons.pistonB_Down)
      {
        digitalWrite(cwDcBPin, LOW);
        digitalWrite(ccwDcBPin, HIGH);
      }
      //PistonB_Up
      else if(buttons.pistonB_Up)
      { 
        digitalWrite(cwDcBPin, HIGH);
        digitalWrite(ccwDcBPin, LOW);
      }
    //Idle
      if(!(buttons.pistonB_Up || buttons.pistonB_Down ||  buttons.pistonF_Up ||  buttons.pistonF_Down))
      {
        digitalWrite(cwDcFPin, LOW);
        digitalWrite(cwDcBPin, LOW);
        digitalWrite(ccwDcFPin, LOW);
        digitalWrite(ccwDcBPin, LOW);
      }

  analogWrite(speedMotor1_pin, SpeedBL);
  analogWrite(speedMotor2_pin, SpeedBR);
  analogWrite(speedMotor3and4_pin, SpeedFLandFR);
  
  digitalWrite(directionMotor1_pin, DirectionBR);
  digitalWrite(directionMotor2_pin, DirectionBL);
  digitalWrite(directionMotor3and4_pin, DirectionFLandFR);

  #if (defined enableSerialprint)
    // Serial.print("SpeedFLandFR: ");
    // Serial.print(SpeedFLandFR);
    // Serial.print(", SpeedBL: ");
    // Serial.print(SpeedBL);
    // Serial.print(", SpeedBR: ");
    // Serial.print(SpeedBR);
    // Serial.print(", buttons.x: ");
    // Serial.print(joystick.x);
    // Serial.print(", buttons.y: ");
    // Serial.print(joystick.y);
    // Serial.print(", DirectionBL: ");
    // Serial.print(DirectionBL);
    // Serial.print(", DirectionBR: ");
    // Serial.print(DirectionBR);

    Serial.print("Up: ");
    Serial.print(buttons.up);
    Serial.print(", Down: ");
    Serial.print(buttons.down);
    Serial.print(", Left: ");
    Serial.print(buttons.left);
    Serial.print(", Right: ");
    Serial.print(buttons.right);
    Serial.print(", PistonF_Down: ");
    Serial.print(buttons.pistonF_Down);
    Serial.print(", PistonF_Up: ");
    Serial.print(buttons.pistonF_Up);
    Serial.print(", PistonB_Down: ");
    Serial.print(buttons.pistonB_Down);
    Serial.print(", PistonB_Up: ");
    Serial.println(buttons.pistonB_Up);
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