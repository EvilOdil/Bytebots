#include <AccelStepper.h>
//stepper motor pins
#define motorPin1  48      // IN1
#define motorPin2  49      // IN2
#define motorPin3  50     // IN3
#define motorPin4  51    // IN4 
#define MotorInterfaceType 4
AccelStepper stepper = AccelStepper(MotorInterfaceType, motorPin1, motorPin3, motorPin2, motorPin4);

void lift(){
  stepper.setCurrentPosition(0);
    while (stepper.currentPosition() != -700) { 
      stepper.setSpeed(-500);
      stepper.runSpeed();
  }

}

void drop(){
stepper.setCurrentPosition(0);
    while (stepper.currentPosition() != 700) { 
      stepper.setSpeed(500);
      stepper.runSpeed();
  }

}

void liftOneBoxHeight(){
  stepper.setCurrentPosition(0);
        while (stepper.currentPosition() != -1500) { 
    stepper.setSpeed(-500);
    stepper.runSpeed();
  }


}

void liftTwoBoxHeight(){
  stepper.setCurrentPosition(0);
        while (stepper.currentPosition() != -3000) { 
    stepper.setSpeed(-500);
    stepper.runSpeed();
  }


}

void setup() {
  stepper.setMaxSpeed(1000);
  stepper.setCurrentPosition(0); //set current pos to 0
  // put your setup code here, to run once:
  delay(1000);
  liftOneBoxHeight();
  delay(1000);
  liftOneBoxHeight();

}

void loop() {
  // put your main code here, to run repeatedly:
   liftTwoBoxHeight();
   delay(1000);
  
  

}
