#include <Arduino.h>

// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib

#include "I2Cdev.h"
#include "Wire.h"
#include "PID_v1.h"

#define enableSerialprint

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define DirectionMotor1_pin 27 //27
#define EnableMotors_pin 12 //12
#define pwmSignal_pin 13 

void disableMotors();
void enableMotors();
double Pid(double error);
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max);

bool blinkState = false;
bool On = false;
bool Direction;
float Speed = 0.0;
float Acc = 0.0;
float Roll = 0.0;
float maxSpeed = 80.0;  //av 255
float treshHold = 0.8;
float activationDegree = 0.15;
long t1;
long t2 = 1;
long deltaT_buttons;
long deltaT_ME;
//PID
double dt, last_time;
double integral, previous, pid = 0;
double kp = 1;
double ki = 0.0; //skippa helt
double kd = 5; //20

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================


void setup()
{
  pinMode(DirectionMotor1_pin, OUTPUT);
  pinMode(EnableMotors_pin, OUTPUT);
  pinMode(pwmSignal_pin, OUTPUT);

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
  
  // if (Roll < 0)
  //   Direction = false;
  // else
  //   Direction = true;

  //Speed = abs(mapfloat(1.3 * (-atan(pid) + pid), -4.5, 4.5, -maxSpeed, maxSpeed)) + treshHold;
  Speed = 40;
  enableMotors();

  if (Speed >= maxSpeed)
    Speed = maxSpeed;
  //if (Speed <= 0)
  //  Speed = 0;
  analogWrite(pwmSignal_pin, 100);

  digitalWrite(DirectionMotor1_pin, Direction);

#if (defined enableSerialprint)

  Serial.print(", Speed: ");
  Serial.print(Speed);
  Serial.print(", Pid: "); 
  Serial.print(pid);
  Serial.print(", Acc: ");
  Serial.print(Acc);
  Serial.print(", Direction: ");
  Serial.print(Direction);
  Serial.print(", dT_ME: ");
  Serial.println(deltaT_ME);
  
#endif

  deltaT_buttons = millis() - t1;
  deltaT_ME = (millis() - t2) / 1000;
  delay(10);
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
