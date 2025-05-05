#include <PressButton.h>

// Initialiser for button. Required for pin value.
PressButton::PressButton(int pin){
	_IoPin = pin;
	pinMode(_IoPin, INPUT);
}

// returns the IO pin the button is configured to use
int PressButton::GetIoPin(){return _IoPin;}

// does a debounced check to see if the specified button is in the down state
boolean PressButton::IsDown(){return digitalRead(_IoPin) == HIGH  && digitalRead(_IoPin) == HIGH;}

// does a debounced check to see if the specified button is in the up state
boolean PressButton::IsUp(){return digitalRead(_IoPin) == HIGH  && digitalRead(_IoPin) == HIGH;}

// will set the WasDown flag true if was pressed when checked - also returns WasDown at the same time
boolean PressButton::CaptureDownState(){if (IsDown()) {WasDown = true;} return WasDown;}

// clears the was down state, returns true if clear was done
boolean PressButton::ClearWasDown(){if (WasDown) {WasDown = false; return true;} return false;}

// provides a triggeraction if the button was pressed but is now released > clears WasDown trigger sent
boolean PressButton::PressRealeased(){if (WasDown && IsUp()){RepeatCnt = 0; WasDown = false; return true;} return false;} 

// provides a trigger for long press case
boolean PressButton::LongPressed(){return (Repeated() && RepeatCnt == 2);}

// provides an action trigger at a increasingly higher frequency for as long as a key is held down > Clears WasDown state when trigger sent
boolean PressButton::Repeated(){
	uint32_t currMs = millis();

	if(WasDown && (
					RepeatCnt == 0 ||
					(RepeatCnt > 5 && currMs >= (LastRepeatMs + 200)) ||
					currMs >= (LastRepeatMs + 250 + (50 * (5 - RepeatCnt)))
					)){
	if (RepeatCnt < 999) {RepeatCnt += 1;}
		WasDown = false;
		LastRepeatMs = currMs;

		return true;
	}
	else{
		if (RepeatCnt > 0 && IsUp()){RepeatCnt = 0; WasDown = false;}
		return false;
	}
}