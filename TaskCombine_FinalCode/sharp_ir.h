
#ifndef SHARP_IR_H
#define SHARP_IR_H

#include <Arduino.h>

const int IR_PINs[2] = {A0, A1};

class SharpIR{
    public:
      int distances[2];

      void read();
      void readSensor(int sensor);

};

#endif