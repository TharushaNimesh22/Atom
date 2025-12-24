#include "distance_sensors.h"

//-------------------------------------- Constructor function---------------------------------
DistanceSensors::DistanceSensors(){
  for (int i = 0; i < NUM_DISTANCE_SENSORS; i++) {
    _vl53Addresses[i] = 0x30 + i;
}
}

// -------------------------Sensors initialization---------------------------------------------
void DistanceSensors::init(int pins[]){
  for (int i = 0; i < NUM_DISTANCE_SENSORS; i++) {
      pinMode(pins[i], OUTPUT);
      digitalWrite(pins[i], LOW);
  }
  delay(50);

  Serial.println("Initializing VL53L0X sensors...");

  // //Select the MUX
  // _tcaSelect(MUX);

  for (int i = 0; i < NUM_DISTANCE_SENSORS; i++) {

    // enable only one sensor
    digitalWrite(pins[i], HIGH);
    delay(20);

    if (!vl53[i].begin()) {
      Serial.print("VL53 INIT FAILED at sensor ");
      Serial.println(i);
      while (1);
    }

    // assign new I2C address
    vl53[i].setAddress(_vl53Addresses[i]);

    Serial.print("VL53 Sensor ");
    Serial.print(i);
    Serial.print(" set to address 0x");
    Serial.println(_vl53Addresses[i], HEX);
  }

  Serial.println("All VL53 sensors initialized!");

}



void DistanceSensors::read(){
  // _tcaSelect(MUX); // all VL53L0X on channel 0

  for (int i = 0; i < NUM_DISTANCE_SENSORS; i++) {
    VL53L0X_RangingMeasurementData_t measure;
    vl53[i].rangingTest(&measure, false);

    Serial.print("VL53[");
    Serial.print(i);
    Serial.print("] = ");

    if (measure.RangeStatus != 4){
        Serial.print(measure.RangeMilliMeter);
      values[i] = measure.RangeMilliMeter;
      }else{
           Serial.print("Out of range");
      values[i] = -1;
      }
     

    Serial.println(" mm");
  }

  Serial.println();
}


// Function to print the distance values
void DistanceSensors::printValues(){
  for (int i = 0; i < NUM_DISTANCE_SENSORS; i++){
    Serial.print("value");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(values[i]);
    Serial.print("; ");

  }

  Serial.println();
  Serial.println();
}
