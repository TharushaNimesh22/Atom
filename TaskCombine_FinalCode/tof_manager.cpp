#include "tof_manager.h"

// Static instance pointer for ISR
TOF_Manager* TOF_Manager::instance = nullptr;

// Constructor
TOF_Manager::TOF_Manager(uint8_t xshutPins[], uint8_t intPin) {
    for (uint8_t i = 0; i < MAX_TOF_SENSORS; i++) {
        _xshutPins[i] = xshutPins[i];
        _assignedAddresses[i] = 0x30 + i;   // Assign addresses 0x30–0x35 (example)
    }
    _intPin = intPin;
    _activeSensorIndex = 255;
    _tofTriggered = false;

    // Set the global instance pointer for the ISR to use
    instance = this;
}

// Reset all sensors before address assignment (pull XSHUT low)
void TOF_Manager::resetAllXSHUT() {
    for (uint8_t i = 0; i < MAX_TOF_SENSORS; i++) {
        pinMode(_xshutPins[i], OUTPUT);
        digitalWrite(_xshutPins[i], LOW);
    }
    delay(20);
}

// Power up one sensor at a time (release XSHUT high)
void TOF_Manager::powerUpSensor(uint8_t index) {
    pinMode(_xshutPins[index], OUTPUT);
    digitalWrite(_xshutPins[index], HIGH);
    delay(20);
}

// Assign new I2C address for sensor at index
// Sequence: power sensor -> begin() (default 0x29) -> setAddress(newAddr)
bool TOF_Manager::assignAddress(uint8_t index, uint8_t newAddr) {
    // begin() uses default address 0x29 internally (Adafruit library)
    if (!_sensors[index].begin()) {
        return false;
    }
    // setAddress is provided by the Adafruit wrapper
    _sensors[index].setAddress(newAddr);
    delay(10);

    return true;
}

// MAIN INITIALIZER — assign addresses for all sensors
bool TOF_Manager::initAllSensors() {
    resetAllXSHUT();

    for (uint8_t i = 0; i < MAX_TOF_SENSORS; i++) {
        powerUpSensor(i);

        if (!assignAddress(i, _assignedAddresses[i])) {
            // failure: could not initialize sensor i
            return false;
        }

        // Ensure sensor is not continuously ranging (make idle)
        // Adafruit library uses stopRangeContinuous()
        _sensors[i].stopRangeContinuous();
        delay(20);
    }

    // Prepare interrupt pin
    pinMode(_intPin, INPUT);

    // Attach ISR (trigger on CHANGE or FALLING/LOW depending on your needs).
    // The ISR is a static function.
    attachInterrupt(digitalPinToInterrupt(_intPin), TOF_Manager::isrHandler, FALLING);

    return true;
}

// Start ONE sensor with interrupt-based continuous mode
bool TOF_Manager::startInterruptSensor(uint8_t sensorIndex,
                                       uint16_t lowThreshMM,
                                       uint16_t highThreshMM)
{
    if (sensorIndex >= MAX_TOF_SENSORS) return false;

    _activeSensorIndex = sensorIndex;

    auto &lox = _sensors[sensorIndex];

    // The sensor was already assigned address earlier; re-init with that address
    // Some Adafruit versions accept begin(addr) — but begin() without args is safe
    if (!lox.begin(_assignedAddresses[sensorIndex])) return false;

    // Configure interrupt GPIO 
    lox.setGpioConfig(
        VL53L0X_DEVICEMODE_CONTINUOUS_RANGING,
        VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_LOW,
        VL53L0X_INTERRUPTPOLARITY_LOW
    );

    FixPoint1616_t low = (lowThreshMM * 65536.0);
    FixPoint1616_t high = (highThreshMM * 65536.0);

    lox.setInterruptThresholds(low, high, true);

    lox.setDeviceMode(VL53L0X_DEVICEMODE_CONTINUOUS_RANGING, false);
    lox.startMeasurement();

    _tofTriggered = false;

    return true;
}

// ISR — sets flag when interrupt occurs
// Must be static with signature void func(void).
void TOF_Manager::isrHandler() {
    // instance pointer allows us to touch non-static members safely
    if (instance) {
        // VL53L0X interrupt is typically active-low for threshold cross
        if (digitalRead(instance->_intPin) == LOW) {
            instance->_tofTriggered = true;
        } else {
            // optionally clear or set false when line goes high
            instance->_tofTriggered = false;
        }
        
    }
}

// Returns true when obstacle detected
bool TOF_Manager::tofObstacleTriggered() {
    return _tofTriggered;
}

// Clear interrupt so next event can be detected
void TOF_Manager::clearInterrupt() {
    if (_activeSensorIndex < MAX_TOF_SENSORS) {
        _sensors[_activeSensorIndex].clearInterruptMask(false);
    }
    _tofTriggered = false;
}

// Get last measurement distance
uint16_t TOF_Manager::getDistance() {
    if (_activeSensorIndex >= MAX_TOF_SENSORS) return 0;

    VL53L0X_RangingMeasurementData_t measure;
    _sensors[_activeSensorIndex].getRangingMeasurement(&measure, false);

    if (measure.RangeStatus == 4)
        return 8191; // Out of range

    return measure.RangeMilliMeter;
}
