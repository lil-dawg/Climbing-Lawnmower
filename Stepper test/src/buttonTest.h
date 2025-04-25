#include <Arduino.h>

const int buttonPin = 14;
 
int ledState = HIGH;         // the current state of the output pin
int buttonState;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

bool readButton(int buttonPin);
 
void buttonSetup() {
  pinMode(buttonPin, INPUT);
  pinMode(ledPin, OUTPUT);
}
 
// void loop() {
//   bool button = readButton(buttonPin);
//   digitalWrite(ledPin, ledState);
// }

bool readButton(int buttonPin) {
  // read the state of the switch into a local variable:
  int reading = digitalRead(buttonPin);
  // If the switch changed, due to noise or pressing:
  if (reading != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }
  
  if ((millis() - lastDebounceTime) > debounceDelay) {
  
    // if the button state has changed:
    if (reading != buttonState) {
      buttonState = reading;
      Serial.println(buttonState);
      // only toggle the LED if the new button state is HIGH
      if (buttonState == HIGH) {
        ledState = !ledState;
      }
    }
  }
  lastButtonState = reading;
  return buttonState;
}