#include <AccelStepper.h>

#define dirPin 2
#define stepPin 3
#define motorInterfaceType 1

AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

void setup() {
  stepper.setMaxSpeed(1000);
}

void loop() { 
  stepper.setCurrentPosition(0);
  while(stepper.currentPosition() != 400)
  {
    stepper.setSpeed(600);
    stepper.runSpeed();
  }
  delay(1000);
  stepper.setCurrentPosition(0);

  while(stepper.currentPosition() != -200) 
  {
    stepper.setSpeed(-600);
    stepper.runSpeed();
  }

  delay(1000);

  stepper.setCurrentPosition(0);

  while(stepper.currentPosition() != 600)
  {
    stepper.setSpeed(600);
    stepper.runSpeed();
  }

  delay(3000);
}

