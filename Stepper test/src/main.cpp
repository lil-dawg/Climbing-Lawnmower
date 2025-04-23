#include <Arduino.h>

// Define button pins
#define BUTTON1 12
#define BUTTON2 13

// Microstepping control pins
#define M0_PIN 25
#define M1_PIN 26
#define M2_PIN 27

// Define stepper pins
#define STEP_PIN 32      // Step pin
#define DIR_PIN 33       // Direction pin

// Steps per revolution for the motor
const float stepsPerRevolution = 200;

void setup() {
  // Set button pins as inputs
  pinMode(BUTTON1, INPUT);
  pinMode(BUTTON2, INPUT);

  // Set microstepping pins as outputs
  pinMode(M0_PIN, OUTPUT);
  pinMode(M1_PIN, OUTPUT);
  pinMode(M2_PIN, OUTPUT);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  // Set microstepping mode (adjust as needed: HIGH or LOW)
  digitalWrite(M0_PIN, LOW);  // Set to LOW or HIGH for desired microstep setting
  digitalWrite(M1_PIN, LOW);  // Set to LOW or HIGH for desired microstep setting
  digitalWrite(M2_PIN, LOW);  // Set to LOW or HIGH for desired microstep setting

	Serial.begin(115200);
  digitalWrite(DIR_PIN, LOW);
}

void loop()
{
	//int delayTime = rpmToDelay(600);
	//Serial.println(delayTime);
  //spinMotor(1, delayTime);

	//delay(1000); // Wait a second
	
	// Set motor direction counterclockwise
	bool buttonState = digitalRead(BUTTON1);
	if (buttonState){
		Serial.println(buttonState);
		digitalWrite(STEP_PIN, HIGH);
		delayMicroseconds(1000);
		digitalWrite(STEP_PIN, LOW);
		delayMicroseconds(1000);
	}

	// Spin motor quickly
	// while(digitalRead(BUTTON1) == HIGH)
	// {
	// 	digitalWrite(STEP_PIN, HIGH);
	// 	delayMicroseconds(1000);
	// 	digitalWrite(STEP_PIN, LOW);
	// 	delayMicroseconds(1000);
	// }
	// delay(50); // Wait a second
}

int rpmToDelay(int rpm)
{
	int delay = 60*pow(10,6)/(rpm*200);
	return delay;
}

void spinMotor(bool dir, int delay)
{
	
  digitalWrite(DIR_PIN, dir);

  for(int x = 0; x < stepsPerRevolution; x++)
	{
		digitalWrite(STEP_PIN, HIGH);
		delayMicroseconds(delay);
		digitalWrite(STEP_PIN, LOW);
		delayMicroseconds(delay);
	}
}