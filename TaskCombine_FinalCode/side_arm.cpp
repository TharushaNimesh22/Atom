#include "side_arm.h"

// // Constructor
// SideArm::SideArm()
// : _stepper(AccelStepper::HALF4WIRE, 8, 10, 9, 11){

// };

// Arm installation
void SideArm::init(){
  // Lift the arm to the initial position
  for (int i = 0; i < 4; i++){
    pinMode(_pins[i], OUTPUT);
  }
  // _moveArm(_upperPosition);
  Serial.println("Side arm is initiated");

  // Define max speed, acceleration, starting speedW
}

// Fubnction to move the arm
void SideArm::moveArm(int direction, int steps){
       // 360°
  for (int i = 0; i < steps; i++) {
    _stepMotor(direction);
  }

}

// Rotate 360 degrees(full turn)
void SideArm::rotateFull(int direction){
    moveArm(direction, numStepsPerRev);
}

// Step one position
void SideArm::_stepMotor(int direction) {
  static int stepIndex = 0;

  for (int pin = 0; pin < 4; pin++) {
    digitalWrite(_pins[pin], _stepSequence[stepIndex][pin]);
  }

  stepIndex += direction;

  if (stepIndex > 7) stepIndex = 0;
  if (stepIndex < 0) stepIndex = 7;

  delay(_stepDelay/2);
}
