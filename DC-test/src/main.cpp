#include <Arduino.h>
#include <PressButton.h>

PressButton button1(14);
PressButton button2(13);

int lamp = 5;

int INT3 = 21;
int INT4 = 22;

void setup()
{
  pinMode(lamp, OUTPUT);
	pinMode(INT3, OUTPUT);
	pinMode(INT4, OUTPUT);
}

void loop()
{
	if (button1.IsDown()) {
		//Serial.println(1);
		digitalWrite(INT4, LOW);
    analogWrite(INT3, 255);
    digitalWrite(lamp, LOW);

	}
	else if (button2.IsDown()) {
		digitalWrite(INT3, LOW);
    analogWrite(INT4, HIGH);
    digitalWrite(lamp, LOW);
	}
	else {
    digitalWrite(INT3, LOW);
    digitalWrite(INT4, LOW);
    digitalWrite(lamp, HIGH);
	}

}