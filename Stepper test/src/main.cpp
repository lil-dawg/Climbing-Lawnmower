#include <Arduino.h>
#include <PressButton.h>

// Define pin connections & motor's steps per revolution
const int dirPin = 33;
const int stepPin = 32;
const int stepsPerRevolution = 200;

PressButton button1(14);

void moveMotor(bool motorDir, int rpm);

void setup()
{
	// Declare pins as Outputs
	pinMode(stepPin, OUTPUT);
	pinMode(dirPin, OUTPUT);

	Serial.begin(115200);
}
void loop()
{
	Serial.println(button1.IsDown());
	if (button1.IsDown()) {
		moveMotor(1, 150);
		delay(1000); // Wait a second
		moveMotor(0, 300);
		delay(1000); // Wait a second
	}

}

void moveMotor(bool motorDir, int rpm)
{
	// Convert rpm to delay time in microseconds
	int delayTime = 60*pow(10,6)/(rpm*200);

	// Set motor direction clockwise
	digitalWrite(dirPin, motorDir);

	// Spin motor slowly
	for(int x = 0; x < stepsPerRevolution; x++)
	{
		digitalWrite(stepPin, HIGH);
		delayMicroseconds(delayTime);
		digitalWrite(stepPin, LOW);
		delayMicroseconds(delayTime);
	}
}