#include <Arduino.h>
#include "I2Cdev.h"
#include "Wire.h"
#include "PID_v1.h"
#include <esp_now.h>
#include <WiFi.h>
#include <AccelStepper.h>

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

struct_message buttons;
struct_message joystick;
//Piston
  //Stepper
    //PistonF
    // AccelStepper stepperF(AccelStepper::DRIVER, 18, 19);  
    // //PistonB
    // AccelStepper stepperB(AccelStepper::DRIVER, 13, 32); 

    // const int dirPinF = 19;
    // const int stepPinF = 18;

    // const int dirPinB = 32;
    // const int stepPinB = 13;
  //DC ----------------------------------
  #define cwDcFPin 19
  #define ccwDcFPin 18
  #define cwDcBPin 32
  #define ccwDcBPin 13

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

#define enableStepperMotor_pin 33

//Joystick
// #define joystickX 21 //temp
// #define joystickY 32 //temp

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
  memcpy(&buttons, incomingData, sizeof(buttons));
  // Serial.print("x: ");
  // Serial.println(buttons.x);
  // Serial.print("y: ");
  // Serial.println(buttons.y);
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

  // pinMode(stepPinF, OUTPUT);
	// pinMode(dirPinF, OUTPUT);
  // pinMode(stepPinB, OUTPUT);
	// pinMode(dirPinB, OUTPUT);

  pinMode(cwDcFPin, OUTPUT);
  pinMode(ccwDcFPin, OUTPUT);
  pinMode(cwDcBPin, OUTPUT);
  pinMode(ccwDcBPin, OUTPUT);

  // pinMode(joystickX, INPUT);
  // pinMode(joystickY, INPUT);

  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock.
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  Serial.begin(115200); //Serial communication

  // stepperF.setMaxSpeed(4000);
  // stepperF.setAcceleration(30);
  // stepperB.setMaxSpeed(4000);
  // stepperB.setAcceleration(30);

  delay(1000);
}

void loop()
{
  // joystick.x = analogRead(joystickX)/4095.0*255.0;
  // joystick.y = analogRead(joystickY)/4095.0*255.0;

  double now = micros();
  dt = (now - last_time)/1000.0;
  last_time = now;

  // pid = Pid(Roll);

 
  // //Joystick mapping
  // //state 1 - Up 
  // if(callibrationX - threshold < joystick.x && joystick.x < callibrationX + threshold && joystick.y > callibrationY + threshold)
  // {
  //   Serial.print("1, ");
  //   SpeedBL = 1.0*255;
  //   SpeedBR = 1.0*255;
  //   DirectionBL = false;
  //   DirectionBR = true;

  //   SpeedFLandFR = joystick.y-callibrationY;
  //   DirectionFLandFR = true;
  // } 
  // //state 2 - Diagonally up-right
  // else if(joystick.x > callibrationX + threshold && joystick.y > callibrationY + threshold)
  // {
  //   Serial.print("2, ");
  //   SpeedBL = 0.5*255;
  //   SpeedBR = 0.5*255;
  //   DirectionBL = false;
  //   DirectionBR = true;

  //   SpeedFLandFR = 0.0*255;
  // }
  // //state 3 - Right
  // else if(joystick.x > callibrationX  + threshold && callibrationY - threshold < joystick.y && joystick.y < callibrationY + threshold)
  // {
  //   Serial.print("3, ");
  //   SpeedBL = 0.75*255;
  //   SpeedBR = 0.75*255;
  //   DirectionBL = false;
  //   DirectionBR = false; 

  //   SpeedFLandFR = 0.0*255;
  // }
  // //state 4 - Diagonally down-right
  // else if(joystick.x > callibrationX + threshold && joystick.y < callibrationY - threshold)
  // {
  //   Serial.print("4, ");
  //   SpeedBL = 0.5*255;
  //   SpeedBR = 1.0*255;
  //   DirectionBL = true;
  //   DirectionBR = false;

  //   SpeedFLandFR = 0.0*255;
  // }
  // //state 5 - Down
  // else if(callibrationX - threshold < joystick.x && joystick.x < callibrationX + threshold && joystick.y < callibrationY - threshold)
  // {
  //   Serial.print("5, ");
  //   SpeedBL = 1.00*255;
  //   SpeedBR = 1.00*255;
  //   DirectionBL = true;
  //   DirectionBR = false; 

  //   SpeedFLandFR = 0.0*255;
  // }
  // //state 6 - Diagonally down-left
  // else if(joystick.x < callibrationX - threshold && joystick.y < callibrationY - threshold)
  // {
  //   Serial.print("6, ");
  //   SpeedBL = 1.00*255;
  //   SpeedBR = 1.00*255;
  //   DirectionBL = true;
  //   DirectionBR = false; 

  //   SpeedFLandFR = 0.0*255;
  // }
  // //state 7 - Left
  // else if(joystick.x < callibrationX - threshold && callibrationY - threshold < joystick.y && joystick.y < callibrationY + threshold)
  // {
  //   Serial.print("7, ");
  //   SpeedBL = 0.75*255;
  //   SpeedBR = 0.75*255;
  //   DirectionBL = true;
  //   DirectionBR = true; 

  //   SpeedFLandFR = 0.0*255;
  // }
  // //state 8 - Diagonally up-left
  // else if(joystick.x < callibrationX - threshold && joystick.y > callibrationY + threshold)
  // {
  //   Serial.print("8, ");
  //   SpeedBL = 0.50*255;
  //   SpeedBR = 1.00*255;
  //   DirectionBL = false;
  //   DirectionBR = true; 

  //   SpeedFLandFR = 0.0*255;
  // }
  // //state 9 - Idle
  // else
  // {
  //   Serial.print("9, ");
  //   SpeedBL = 0.0*255;
  //   SpeedBR = 0.0*255;
  //   DirectionBL = false;
  //   DirectionBR = true;
    
  //   SpeedFLandFR = 0.0*255;
  // }

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
    Serial.println("Right");
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

  awakeMotors();
  
  if (SpeedBR >= maxSpeed)
    SpeedBR = maxSpeed;
  
  if (SpeedBL >= maxSpeed)
    SpeedBL = maxSpeed;

//Pistons -----------------------------------------------------------------
    //PistonF
    if (buttons.pistonF_Down) {
      digitalWrite(cwDcFPin, LOW);
      digitalWrite(ccwDcFPin, HIGH);
    }
    else if (buttons.pistonF_Up) {
      digitalWrite(cwDcFPin, HIGH);
      digitalWrite(ccwDcFPin, LOW);
    }
    //PistonB
    if(buttons.pistonB_Down)
    {
      digitalWrite(cwDcBPin, LOW);
      digitalWrite(ccwDcBPin, HIGH);
    }
    else if(buttons.pistonB_Up)
    { 
      digitalWrite(cwDcBPin, HIGH);
      digitalWrite(ccwDcBPin, LOW);
    }
    if(!(buttons.pistonB_Up || buttons.pistonB_Down ||  buttons.pistonF_Up ||  buttons.pistonF_Down))
    {
      digitalWrite(cwDcFPin, LOW);
      digitalWrite(cwDcBPin, LOW);
      digitalWrite(ccwDcFPin, LOW);
      digitalWrite(ccwDcBPin, LOW);
    }

  // //Pistons -----------------------------------------------------------------
  //   //PistonF
  //   if (buttons.pistonF_Down) {
  //     digitalWrite(enableStepperMotor_pin, LOW); //enableStepperMotor_pin
  //     // stepperF.setSpeed(1000);
  //     // stepperF.runSpeed();
  //     digitalWrite(dirPinF, HIGH);
  //     digitalWrite(stepPinF, HIGH);
  //     delayMicroseconds(8000);
  //     digitalWrite(stepPinF, LOW);
  //     delayMicroseconds(8000);
  //   }
  //   else if (buttons.pistonF_Up) {
  //     digitalWrite(enableStepperMotor_pin, LOW); //enableStepperMotor_pin
  //     // stepperF.setSpeed(-1000);
  //     // stepperF.runSpeed();
  //     digitalWrite(dirPinF, LOW);
  //     digitalWrite(stepPinF, HIGH);
  //     delayMicroseconds(8000);
  //     digitalWrite(stepPinF, LOW);
  //     delayMicroseconds(8000);
  //   }
  //   //PistonB
  //   if(buttons.pistonB_Down)
  //   {
  //     digitalWrite(enableStepperMotor_pin, LOW); //enableStepperMotor_pin
  //     // stepperB.setSpeed(1000);
  //     // stepperB.runSpeed();
  //     digitalWrite(dirPinB, HIGH);
  //     digitalWrite(stepPinB, HIGH);
  //     delayMicroseconds(8000);
  //     digitalWrite(stepPinB, LOW);
  //     delayMicroseconds(8000);
  //   }
  //   else if(buttons.pistonB_Up)
  //   {
  //     digitalWrite(enableStepperMotor_pin, LOW); //enableStepperMotor_pin
  //     // stepperB.setSpeed(-1000);
  //     // stepperB.runSpeed();
  //     digitalWrite(dirPinB, LOW);
  //     digitalWrite(stepPinB, HIGH);
  //     delayMicroseconds(8000);
  //     digitalWrite(stepPinB, LOW);
  //     delayMicroseconds(8000);
  //   }
  //   if(!(buttons.pistonB_Up || buttons.pistonB_Down ||  buttons.pistonF_Up ||  buttons.pistonF_Down))
  //   {
  //     digitalWrite(enableStepperMotor_pin, HIGH); //disableStepperMotor_pin
  //   }

  analogWrite(speedMotor1_pin, SpeedBL);
  analogWrite(speedMotor2_pin, SpeedBR);
  digitalWrite(directionMotor1_pin, DirectionBR);
  digitalWrite(directionMotor2_pin, DirectionBL);

  analogWrite(speedMotor3and4_pin, SpeedFLandFR);
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