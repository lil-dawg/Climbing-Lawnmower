#include <Arduino.h>

const int dirPin = 26;
const int stepPin = 27;

void setup()
{
	pinMode(stepPin, OUTPUT);
	pinMode(dirPin, OUTPUT);
}
void loop()
{
	// Set motor direction clockwise
	digitalWrite(dirPin, HIGH);
	digitalWrite(stepPin, HIGH);
	delayMicroseconds(1000);
	digitalWrite(stepPin, LOW);
	delayMicroseconds(1000);
	// Set motor direction counterclockwise
	digitalWrite(dirPin, LOW);
	digitalWrite(stepPin, HIGH);
	delayMicroseconds(1000);
	digitalWrite(stepPin, LOW);
	delayMicroseconds(1000);

}