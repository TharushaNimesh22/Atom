#include "line_follower.h"

// Constructor
LineFollower::LineFollower()
{
    baseSpeed = 100;
    steering  = 0;
    lastTime  = 0;
}

// INITIALIZATION
void LineFollower::init()
{
    // Initialize hardware
    irsensors.init();
    steppers.init();
    tracker.init();

    // Configure PID constants
    pid.Kp = 55.5f;
    pid.Ki = 0.0f;
    pid.Kd = 0.0f;

    pid.tau = 0.02f;

    pid.limMax =  200;
    pid.limMin = -200;
    pid.limMaxInt = 50;
    pid.limMinInt = -50;

    pid.T = LOOP_DT;
    pid.init();

    lastTime = millis();
}

// MAIN LOOP UPDATE
void LineFollower::update(bool followDir)
{
    // Run PID loop every 10 ms
    if (millis() - lastTime < LOOP_DT * 1000) return;
    lastTime += LOOP_DT * 1000;

    // Read IR array
    irsensors.read();

    // Compute line position
    float pos = tracker.computePosition(irsensors.values);

    // Handle line lost
    if (pos == 999.0f) {
        handleLineLost();
        return;
    }

    // --- PID Steering ---
    float setpoint = 0.0f;        // ideal center
    steering = pid.update(setpoint, pos);

    // Adjust steering of the robot
    float left  = baseSpeed + steering;
    float right = baseSpeed - steering;

    clampMotorSpeeds(left, right);

    // Drive robot
    if (followDir) {
        steppers.setSpeed(right, left);
        steppers.runMotors();
    }

    else {
        steppers.setSpeed(-right, -left);
        steppers.runMotors();
    }
    
}

// HELPERS

// What to do when the line is lost
void LineFollower::handleLineLost()
{
    steppers.setSpeed(+100, -100);   // slow rotation to find line
    pid.integrator = 0;              // avoid windup
    steppers.runMotors();
}

// Clamp values to motor limits
void LineFollower::clampMotorSpeeds(float& left, float& right)
{
    if (left > 255)  left = 255;
    if (left < -255) left = -255;
    if (right > 255)  right = 255;
    if (right < -255) right = -255;
}
