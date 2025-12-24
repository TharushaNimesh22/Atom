#ifndef TOF_MANAGER_H
#define TOF_MANAGER_H

#include <Arduino.h>
#include <Adafruit_VL53L0X.h>

#define MAX_TOF_SENSORS 6

class TOF_Manager {
public:
    TOF_Manager(uint8_t xshutPins[], uint8_t intPin);

    // Initialize: set addresses + stop other sensors
    bool initAllSensors();

    // Start one sensor with interrupt-based continuous ranging
    bool startInterruptSensor(uint8_t sensorIndex,
                              uint16_t lowThreshMM,
                              uint16_t highThreshMM);

    // Returns true when interrupt event triggered
    bool tofObstacleTriggered();

    // Clear interrupt (must be called after event)
    void clearInterrupt();

    // Get last measured distance
    uint16_t getDistance();

    //////////////////////////////////////////////////////////////
    // ISR must be static and have C-compatible signature for attachInterrupt
    static void isrHandler();
    static TOF_Manager* instance;

private:
    void resetAllXSHUT();
    void powerUpSensor(uint8_t index);
    bool assignAddress(uint8_t index, uint8_t newAddress);

    uint8_t _xshutPins[MAX_TOF_SENSORS];
    uint8_t _intPin;

    uint8_t _assignedAddresses[MAX_TOF_SENSORS];
    Adafruit_VL53L0X _sensors[MAX_TOF_SENSORS];

    volatile bool _tofTriggered;
    uint8_t _activeSensorIndex;

    
};

#endif
