#include "wall_tracker.h"

WALLtracker::WALLtracker() {
    dF = dB = 0;
    distanceError = 0;
    angleError = 0;
    combinedVertualMeasurement = 0;
}

// Initialise TOF sensors and PID
void WALLtracker::init(float targetDistance, float k1, float k2, float baseline) {
    // _frontSensor->begin();
    // _backSensor->begin();
    // _pid->init();
    _targetDistance = targetDistance;
    _k1 = k1;
    _k2 = k2;
    _baseline = baseline;
    Serial.println("WaLLtracker..");
}

uint16_t WALLtracker::getFrontDistance() {
    return dF;
}

uint16_t WALLtracker::getBackDistance() {
    return dB;
}

float WALLtracker::getDistanceError() {
    return distanceError;
}

float WALLtracker::getAngleError() {
    return angleError;
}

float WALLtracker::getCombinedError() {
    return combinedVertualMeasurement;
}

// Read the sensor readings and input to this function
float WALLtracker::computeCombinedVertualMeasurement(int readings[]) {

    // Read sensors
    dF = readings[2];
    dB = readings[5];

    Serial.print(dF);
    Serial.print(" ");
    Serial.println(dB);

    // Calculate distance error
    float avgDist = (dF + dB) / 2.0;
    distanceError = _targetDistance - avgDist;

    // Calculate angle error
    angleError = atan( (dF - dB) / _baseline );

    // Combine both errors
    combinedVertualMeasurement = ( _k1 * distanceError + _k2 * angleError );

    // float setpoint = 0;

    // Update the PID
    // float steering = _pid->update(setpoint, combinedVertualMeasurement);  
    return combinedVertualMeasurement;
}


