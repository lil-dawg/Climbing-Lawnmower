// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "PID_v1.h"

MPU6050 mpu;

#define enableSerialprint

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define ledRed_pin 4//15
#define led1_pin 5//32
#define led2_pin 6//33
#define button1_pin 7//25
#define button2_pin 8//26
#define DirectionMotor1_pin 9//27
#define DirectionMotor2_pin 10//14
#define EnableMotors_pin 12 //12
#define pwmSignal_pin 11 //13 esp32

bool blinkState = false;
bool On = false;
bool Direction;
bool b1;
bool b2;
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

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

void setup()
{
  pinMode(ledRed_pin, OUTPUT);
  pinMode(led1_pin, OUTPUT);
  pinMode(led2_pin, OUTPUT);
  pinMode(button1_pin, INPUT);
  pinMode(button2_pin, INPUT);
  pinMode(DirectionMotor1_pin, OUTPUT);
  pinMode(DirectionMotor2_pin, OUTPUT);
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

  // initialize device
  //Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  //Serial.println(F("Testing device connections..."));
  //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  //Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXAccelOffset(-1736);
  mpu.setYAccelOffset(379);
  mpu.setZAccelOffset(947);
  mpu.setXGyroOffset(104);
  mpu.setYGyroOffset(114);
  mpu.setZGyroOffset(20);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    //mpu.CalibrateAccel(6);
    //mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    //Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    //Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    //Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    //Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    //Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    //Serial.print(F("DMP Initialization failed (code "));
    //Serial.print(devStatus);
    //Serial.println(F(")"));
  }

  // configure LED for output
  //pinMode(LED_PIN, OUTPUT);
}

void loop()
{
  //if (!dmpReady) return;
  mpu.dmpGetCurrentFIFOPacket(fifoBuffer);
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  
  double now = micros();
  dt = (now - last_time)/1000.0;
  last_time = now;
  Roll = -ypr[2] * 180 / M_PI;
  pid = Pid(Roll);
  
  if (Roll < 0)
    Direction = false;
  else
    Direction = true;

  b1 = digitalRead(button1_pin);
  b2 = digitalRead(button2_pin);
  if (b1)
    digitalWrite(led1_pin, HIGH);
  else
    digitalWrite(led1_pin, LOW);
  if (b2)
    digitalWrite(led2_pin, HIGH);
  else
    digitalWrite(led2_pin, LOW);

  if (Roll >= -activationDegree && Roll <= activationDegree)
  {
    On = true;
    if (deltaT_ME > 0 && deltaT_ME % 2 == 0)
    {
      disableMotors();
      t2 = millis();
    }
    digitalWrite(ledRed_pin, HIGH);
  }
  else
    digitalWrite(ledRed_pin, LOW);
  //On = true; //test
  if ((!b1 || !b2) && deltaT_buttons > 200)
    On = false;
  if (b1 && b2)
    t1 = millis();

  if (On)
  {
    Speed = abs(mapfloat(1.3 * (-atan(pid) + pid), -4.5, 4.5, -maxSpeed, maxSpeed)) + treshHold;
    enableMotors();
  }
  else
  {
    Speed = 0;
    disableMotors();
  }
  if (Speed >= maxSpeed)
    Speed = maxSpeed;
  //if (Speed <= 0)
  //  Speed = 0;
  analogWrite(pwmSignal_pin, Speed);

  digitalWrite(DirectionMotor1_pin, Direction);
  digitalWrite(DirectionMotor2_pin, !Direction);

#if (defined enableSerialprint)

  Serial.print("\tRoll: ");
  Serial.print(Roll);
  Serial.print(", Speed: ");
  Serial.print(Speed);
  Serial.print(", Pid: "); 
  Serial.print(pid);
  Serial.print(", Acc: ");
  Serial.print(Acc);
  Serial.print(", Direction: ");
  Serial.print(Direction);
  Serial.print(", B1: ");
  Serial.print(b1);
  Serial.print(", B2: ");
  Serial.print(b2);
  Serial.print(", On: ");
  Serial.print(On);
  Serial.print(", dTbuttons: ");
  Serial.print(deltaT_buttons);
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
