#include <Arduino.h>

// Define pin connections & motor's steps per revolution
const int dirPin = 26;
const int stepPin = 27;
const int stepsPerRevolution = 200;

void setup()
{
	// Declare pins as Outputs
	pinMode(stepPin, OUTPUT);
	pinMode(dirPin, OUTPUT);
}
void loop()
{
	// Set motor direction clockwise
	digitalWrite(dirPin, HIGH);

	// Spin motor 4 revolutions slowly
	for(int x = 0; x < 4 * stepsPerRevolution; x++)
	{
		digitalWrite(stepPin, HIGH);
		delayMicroseconds(1000);
		digitalWrite(stepPin, LOW);
		delayMicroseconds(1000);
	}
	
	
	// Set motor direction counterclockwise
	digitalWrite(dirPin, LOW);

	// Spin motor 4 revolutions quickly
	for(int x = 0; x < 4 * stepsPerRevolution; x++)
	{
		digitalWrite(stepPin, HIGH);
		delayMicroseconds(1000);
		digitalWrite(stepPin, LOW);
		delayMicroseconds(1000);
	}
	
}