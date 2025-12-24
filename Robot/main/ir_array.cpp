#include "ir_array.h"

// Constructor: assign pins
IRarray::IRarray() {
    sensorPins[0] = 28;
    sensorPins[1] = 29;
    sensorPins[2] = 30;
    sensorPins[3] = 31;
    sensorPins[4] = 32;
}

// Set pin modes
void IRarray::init() {
    for (int i = 0; i < NUM_SENSORS; i++) {
        pinMode(sensorPins[i], INPUT);
    }
}

// Read all sensors (digital)
void IRarray::read() {
    for (int i = 0; i < NUM_SENSORS; i++) {
        values[i] = digitalRead(sensorPins[i]);

    }
}

// Print sensor values when debugging ( Optional )
void IRarray::print(const int values[]) {
    Serial.print("Sensors: ");
    for (int i = 0; i < NUM_SENSORS; i++) {
        Serial.print(values[i]);
        Serial.print(" ");
    }
    Serial.println();
}
