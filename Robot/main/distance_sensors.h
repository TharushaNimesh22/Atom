#ifndef DISTANCE_SENSORS_H
#define DISTANCE_SENSORS_H

#include <Wire.h>
#include "Adafruit_VL53L0X.h"


constexpr int NUM_DISTANCE_SENSORS = 6;

class DistanceSensors {
  public:
      //Constructor
      DistanceSensors();
      void init(int pins[]);

      //Values array
      int values[NUM_DISTANCE_SENSORS];

      //Read sensor values and update the values array
      void read();

      //optional
      void printValues();

  private:
    //   int _xshutPins[3];
      byte _vl53Addresses[NUM_DISTANCE_SENSORS];
      Adafruit_VL53L0X vl53[NUM_DISTANCE_SENSORS];

};

#endif