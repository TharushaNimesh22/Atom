#include "color_sensors.h"

// Wire initialization for LEDs
void ColorSensors::init1(int red, int green, int blue){
  pinMode(red, OUTPUT);
  pinMode(blue, OUTPUT);
  pinMode(green, OUTPUT);

  _LEDPins[0] = red;
  _LEDPins[1] = green;
  _LEDPins[2] = blue;
}

// Individual sensor initialization
void ColorSensors::init2(int sensor){
  if(!_tcs[sensor].begin()){
    Serial.print("TCS3472 FAILED at channel ");
    Serial.println(sensor + 2);
    // while (1);
  }

  Serial.print("TCS3472 OK at channel ");
  Serial.println(sensor + 2);
}


// Function to read the values of one sensor
    // Return;
    //   RED - 0
    //   GREEN - 1
    //   BLUE - 2
    //   MIXED - -1
int ColorSensors::readSensor(int sensor, int th){
  uint16_t r, g, b, c;
    _tcs[sensor].getRawData(&r, &g, &b, &c);

    Serial.print("TCS[");
    Serial.print(sensor);
    Serial.println("]:");

    Serial.print("  R="); Serial.println(r);
    Serial.print("  G="); Serial.println(g);
    Serial.print("  B="); Serial.println(b);
    Serial.print("  C="); Serial.println(c);

    int color;

    if (r > b && r > g && r >= th){
      Serial.println("Color: RED");
      color = 0;

    }else if(g > r && g > b && g >= th){
      Serial.println("Color: GREEN");
      color = 1;
      
    }else if(b > r && b > g && b >= th){
      Serial.println("Color: BLUE");
      color = 2;

    }else{
      Serial.println("Mixed Color");
      color = -1;
    }

    return color;
}


// Function to read the values of one sensor
    // Return;
    //   RED - 0
    //   GREEN - 1
    //   BLUE - 2
    //   MIXED - -1
int ColorSensors::readSensorDisplay(int sensor, int th){
  // Read the sensor and detect the color
  int color = readSensor(sensor, th);


  // LED display
  for (int i = 0; i < 3; i++){
    if (i == color){
        digitalWrite(_LEDPins[i], HIGH);
        continue;
    }

    digitalWrite(_LEDPins[i], LOW);
  }

  return color;
  
}