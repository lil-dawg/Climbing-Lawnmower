#include <Arduino.h>
#include <PressButton.h>

PressButton button1(14);
PressButton button2(13);

int lamp = 5;

int IN1 = 21;
int IN2 = 22;

void setup()
{
  pinMode(lamp, OUTPUT);
	pinMode(IN1, OUTPUT);
	pinMode(IN2, OUTPUT);
}

void loop()
{
	if (button1.IsDown()) {
		//Serial.println(1);
		digitalWrite(IN1, LOW);
    digitalWrite(lamp, LOW);

	}
	else if (button2.IsDown()) {
		digitalWrite(IN2, LOW);
    digitalWrite(lamp, LOW);
	}
	else {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, HIGH);
    digitalWrite(lamp, HIGH);
	}

}