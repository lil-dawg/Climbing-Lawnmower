#include <PressButton.h>
#include <AccelStepper.h>

PressButton button1(34);
PressButton button2(35);


AccelStepper stepper(AccelStepper::DRIVER, 32, 33);  //StepPin D2 and DirPin D3

void setup()
{  
  stepper.setMaxSpeed(4000);
  stepper.setSpeed(1000);    //One full rotation = 800 steps = 200 steps * 4 (MS1=0, MS2=1, MS3=0)
}

void loop()
{
  if (button1.IsDown()) {
    stepper.setSpeed(1000);
    stepper.runSpeed();
  }
  else if (button2.IsDown()) {
    stepper.setSpeed(-1000);
    stepper.runSpeed();
  }
}