#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController {
public:
    // Gains
    float Kp;
    float Ki;
    float Kd;

    // Derivative low-pass filter time constant
    float tau;

    // Output limits
    float limMin;
    float limMax;

    // Integrator limits
    float limMinInt;
    float limMaxInt;

    // Sample time
    float T;

    // Internal state
    float integrator;
    float prevError;
    float differentiator;
    float prevMeasurement;

    // Output
    float out;

    // Constructor
    PIDController();

    // Methods
    void init();
    float update(float setpoint, float measurement);
};

#endif
