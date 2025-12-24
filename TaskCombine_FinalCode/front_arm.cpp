#include "front_arm.h"

void FrontArm::init(int pin){
  _myServo.attach(pin);
  Serial.println("Front arm is initiated..");
  // _pos = FRONT_ARM_INITIAL_POSITION;
  // _myServo.write(_pos);
  
}

// Get the Arm to the initial position, call this one before using the front arm
void FrontArm::initialPosition(){
    _pos = FRONT_ARM_INITIAL_POSITION;
    _myServo.write(_pos);
}


void FrontArm::armForward(int pos){
  if (_pos > pos) return;
  for (; _pos <= pos; _pos++){
      _myServo.write(_pos);
      delay(10);
    }

}



void FrontArm::armBackward(int pos){
   if (_pos < pos) return;
    for (; _pos >= pos; _pos--){
      _myServo.write(_pos);
      delay(10);
    }
}

void FrontArm::releaseBox(){
    armBackward(FRONT_ARM_INITIAL_POSITION);
}

void FrontArm::catchBox(){
    armForward(180);

}

// Function to print the current position of the arm (optional)
void FrontArm::printPosition(){
  Serial.println(_pos);
}