#include "sharp_ir.h"

void SharpIR::read(){
    int sensorValue;
    float voltage;
    float distance;

    for (int i = 0; i < 2; i++){
      sensorValue = analogRead(IR_PINs[i]);
      voltage = sensorValue * (5.0 / 1023.0);

      distance = 27.728 * pow(voltage, -1.2045);

      distances[i] = distance;
    }
}


void SharpIR::readSensor(int sensor){
  int sensorValue = analogRead(IR_PINs[sensor]);
  float voltage = sensorValue * (5.0 / 1023.0);
  float distance = 27.728 * pow(voltage, -1.2045);

  distances[sensor] = distance;
}