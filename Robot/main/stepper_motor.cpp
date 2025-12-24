#include "stepper_motor.h"

// Constructor initializes stepper objects
STEPPERcontroller::STEPPERcontroller()
: _stepperX(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN),
  _stepperY(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN)
{
}

void STEPPERcontroller::init() {
    // Enable drivers
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, LOW);   

    // Set default parameters
    _stepperX.setMaxSpeed(1000);
    _stepperY.setMaxSpeed(1000);

    // Setting the acceleration
    _stepperX.setAcceleration(50);   
    _stepperY.setAcceleration(50);

    _stepperX.setSpeed(0);
    _stepperY.setSpeed(0);
}

void STEPPERcontroller::setSpeed(float speedX, float speedY) {
    _stepperX.setSpeed(speedX);
    _stepperY.setSpeed(-speedY);
}

void STEPPERcontroller::runMotors() {
    // Non-blocking continuous stepping
    _stepperX.runSpeed();
    _stepperY.runSpeed();
}

void STEPPERcontroller::turn(float left, float right, int steps){
  for (int i = 0; i < steps; i++){
    setSpeed(left, right);
    runMotors();
    delay(5);
  }
}

void STEPPERcontroller::runForward(float speed, int steps){
  turn(speed, speed, steps);
}

void STEPPERcontroller::runSteps(long stepsX, long stepsY) {
  // Run motors to the desired position using acceleration and deceleration
  _stepperX.moveTo(stepsX);
  _stepperX.moveTo(stepsX);

  //Run the motors until finished
  while (_stepperX.distanceToGo() != 0 || _stepperY.distanceToGo() != 0) {
    _stepperX.run();
    _stepperY.run();
  }
}

void STEPPERcontroller::stop() {
    _stepperX.setSpeed(0);
    _stepperY.setSpeed(0);
}
