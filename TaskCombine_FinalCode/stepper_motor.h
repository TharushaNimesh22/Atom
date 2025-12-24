#ifndef STEPPER_MOTOR_H
#define STEPPER_MOTOR_H

#include <Arduino.h>
#include <AccelStepper.h>

// CNC Shield default A4988 pin mappings
#define X_STEP_PIN 2
#define X_DIR_PIN 5
#define Y_STEP_PIN 3
#define Y_DIR_PIN 6
#define ENABLE_PIN 8  

#define S1_STEPS_FOR_90_TURN 140
#define S2_STEPS_FOR_90_TURN 140

class STEPPERcontroller {
public:
    // Constructor
    STEPPERcontroller();

    void init();                          
    void setSpeed(float speedX, float speedY);

    // Move motors 1-step in setSpeed
    void runMotors();

    // Move motors to a desired position ( Optional| Not needed for PID control )
    void runSteps(long stepsX, long stepsY);  

    // Motor turn
    void turn(float left, float right, int steps);

    void runForward(float speed, int steps);

    // Stop both motors
    void stop();         

    // Turn right 90 degrees
    void turnRight90();
    
    // Turn left 90 degrees
    void turnLeft90();                    

private:
    AccelStepper _stepperX;
    AccelStepper _stepperY;
};

#endif
