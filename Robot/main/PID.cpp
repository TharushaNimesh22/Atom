#include "PID.h"

// Constructor: initialize everything to zero
PIDController::PIDController() :
    Kp(0.0f), Ki(0.0f), Kd(0.0f),
    tau(0.0f),
    limMin(0.0f), limMax(0.0f),
    limMinInt(0.0f), limMaxInt(0.0f),
    T(0.0f),
    integrator(0.0f),
    prevError(0.0f),
    differentiator(0.0f),
    prevMeasurement(0.0f),
    out(0.0f)
{
}

void PIDController::init() {
    integrator = 0.0f;
    prevError = 0.0f;
    differentiator = 0.0f;
    prevMeasurement = 0.0f;
    out = 0.0f;
}

float PIDController::update(float setpoint, float measurement) {

    // Error
    float error = setpoint - measurement;

    // Proportional
    float proportional = Kp * error;

    // Integral (Trapezoidal rule)
    integrator += 0.5f * Ki * T * (error + prevError);

    // Anti-windup: clamp integrator
    if (integrator > limMaxInt)
        integrator = limMaxInt;
    else if (integrator < limMinInt)
        integrator = limMinInt;

    // Derivative (band-limited differentiator)
    differentiator = -(2.0f * Kd * (measurement - prevMeasurement)
                    + (2.0f * tau - T) * differentiator)
                    / (2.0f * tau + T);

    // Output
    out = proportional + integrator + differentiator;

    // Output limits
    if (out > limMax)
        out = limMax;
    else if (out < limMin)
        out = limMin;

    // Store variables
    prevError = error;
    prevMeasurement = measurement;

    return out;
}
  