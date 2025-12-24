#ifndef IR_ARRAY_H
#define IR_ARRAY_H

#include <Arduino.h>

// Sensor Count
#define NUM_SENSORS 5

class IRarray {
public:
    bool values[NUM_SENSORS];

    // Constructor
    IRarray();                     

    //Initialise sensor pins
    void init();                   
    void read();       
    void print(const int values[]);

private:
    int sensorPins[NUM_SENSORS];   // internal pin list
};

#endif