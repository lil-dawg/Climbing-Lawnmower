#include <Arduino.h>
#include "I2Cdev.h"
#include "Wire.h"
#include "PID_v1.h"

#define enableSerialprint

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define DirectionMotor1_pin 27 
#define EnableMotors_pin 12 
#define pwmSignal_pin 13 
#define speedControl_pin 14 

void disableMotors();
void enableMotors();
double Pid(double error);
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max);

bool On = false;
bool Direction;
float Speed = 0.0;
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

void setup()
{
  pinMode(DirectionMotor1_pin, OUTPUT);
  pinMode(EnableMotors_pin, OUTPUT);
  pinMode(pwmSignal_pin, OUTPUT);
  pinMode(speedControl_pin, INPUT_PULLDOWN);

  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(9600);
  pinMode(INTERRUPT_PIN, INPUT);
}


void loop()
{
  double now = micros();
  dt = (now - last_time)/1000.0;
  last_time = now;

  // pid = Pid(Roll);
  

  Direction = true;

  //Speed = abs(mapfloat(1.3 * (-atan(pid) + pid), -4.5, 4.5, -maxSpeed, maxSpeed)) + treshHold;
  Speed = analogRead(speedControl_pin)/4095.0*255.0;
  enableMotors();

  if (Speed >= maxSpeed)
    Speed = maxSpeed;
  //if (Speed <= 0)
  //  Speed = 0;
  analogWrite(pwmSignal_pin, Speed);

  digitalWrite(DirectionMotor1_pin, Direction);

#if (defined enableSerialprint)

  Serial.print(", Speed: ");
  Serial.print(Speed);
  Serial.print(", Pid: "); 
  Serial.print(pid);
  Serial.print(", Acc: ");
  Serial.print(Acc);
  Serial.print(", Direction: ");
  Serial.println(Direction);

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
