#ifndef WALL_TRACKER_H
#define WALL_TRACKER_H

#include <Arduino.h>
// #include "distance_sensors.h"
#include "PID.h"

class WALLtracker {
public:
    // Constructor
    WALLtracker(
        // TCA_VL53L0X* frontSensor,
        // TCA_VL53L0X* backSensor,
        // PIDController* pidController
    );

    void init(float targetDistance, float k1, float k2, float baseline);
    // Compute distance and angle errors
    float computeCombinedVertualMeasurement(int readings[]);   

    uint16_t getFrontDistance();
    uint16_t getBackDistance();
    float getDistanceError();
    float getAngleError();
    float getCombinedError();

private:
    // TCA_VL53L0X* _frontSensor;
    // TCA_VL53L0X* _backSensor;
    // PIDController* _pid;

    float _targetDistance;
    float _k1;
    float _k2;
    float _baseline;

    uint16_t dF, dB;          
    float distanceError;
    float angleError;
    float combinedVertualMeasurement;
};

#endif
