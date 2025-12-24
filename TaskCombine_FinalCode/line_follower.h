#ifndef LINE_FOLLOWER_H
#define LINE_FOLLOWER_H

#include <Arduino.h>
#include "PID.h"
#include "ir_array.h"
#include "line_tracker.h"
#include "stepper_motor.h"

class LineFollower {
public:
    // Constructor
    LineFollower();

    void init();     // Initialize hardware + PID

    // Foeward and Reverse line following 
    void update(bool followDir);   // Run one control loop iteration

private:
    // Modules
    PIDController pid;
    IRarray irsensors;
    LINEtracker tracker;
    STEPPERcontroller steppers;

    // Control variables
    float baseSpeed;
    float steering;

    // Timing
    unsigned long lastTime;
    // Change PID loop frequency
    const float LOOP_DT = 0.005f;   

    // Internal helpers
    void handleLineLost();
    void clampMotorSpeeds(float& left, float& right);
};

#endif
