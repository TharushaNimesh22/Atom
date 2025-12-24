#ifndef COLOR_SENSORS_H
#define COLOR_SENSORS_H

#include <Wire.h>
#include "Adafruit_TCS34725.h"

constexpr int NUM_COLOR_SENSORS = 3;

class ColorSensors{
  public:
    // int values[NUM_COLOR_SENSORS][4];
    // Public methods
    void init1(int red, int green, int blue);
    void init2(int sensor);

    // Function to read the values of one sensor
    // Return;
    //   RED - 0
    //   GREEN - 1
    //   BLUE - 2
    //   MIXED - -1
    int readSensor(int sensor, int th);   // int sensor : 0, 1, 2
    int readSensorDisplay(int sensor, int th);

    // //Optional
    // void readAllSensors();
    // int detectColor()

  private:
    // Private attributes
    // int _values[NUM_COLOR_SENSORS][4];

    Adafruit_TCS34725 _tcs[NUM_COLOR_SENSORS];
    int _LEDPins[3];
};


#endif