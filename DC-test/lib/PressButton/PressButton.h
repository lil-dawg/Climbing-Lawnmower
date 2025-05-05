#ifndef Pressbutton_h
#define Pressbutton_h

#include <Arduino.h>

/*
* @breif Multi-function and debounce button object
*/

class PressButton{
private:
	int _IoPin;							// internal value - IO pin

public:
	PressButton(int pin);				// Initialiser for button. Required for pin value.
	int GetIoPin();						// returns the IO pin the button is configured to use
	boolean WasDown = false;			// stores was down state as set by CaptureDownState()
	uint32_t RepeatCnt = 0;				// maintains a count of number of times repeat trigger sent
	uint32_t LastRepeatMs = 0;			// keeps a record of when last repeat trigger was sent
	boolean IsDown();					// does a debounced check to see if the specified button is in the down state
	boolean IsUp();						// does a debounced check to see if the specified button is in the up state
	boolean CaptureDownState();			// will set the WasDown flag true if was pressed when checked - also returns WasDown at the same time
	boolean ClearWasDown();				// clears the was down state, returns true if clear was done
	boolean PressRealeased(); 			// provides a triggeraction if the button was pressed but is now released > clears WasDown trigger sent
	boolean LongPressed();				// provides a trigger for long press case
	boolean Repeated();					// provides an action trigger at a increasingly higher frequency for as long as a key is held down > Clears WasDown state when trigger sent
};

#endif