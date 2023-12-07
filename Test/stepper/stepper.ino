#include <AccelStepper.h>
//stepper motor pins
#define motorPin1  48      // IN1
#define motorPin2  47      // IN2 //49
#define motorPin3  49     // IN3 //50
#define motorPin4  50    // IN4 //51
#define MotorInterfaceType 4
AccelStepper stepper = AccelStepper(MotorInterfaceType, motorPin1, motorPin3, motorPin2, motorPin4);

void lift(){
  stepper.setCurrentPosition(0);
    while (stepper.currentPosition() != -2000) { 
      stepper.setSpeed(-500);
      stepper.runSpeed();
  }

}

void drop(){
stepper.setCurrentPosition(0);
    while (stepper.currentPosition() != 3000) { 
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
  
  lift();
  drop();
 

}

void loop() {
  // put your main code here, to run repeatedly:
 
  
  

}
