// #include <Servo.h>
#include <Wire.h>
#include "robot.h"

// Create robot object
Robot atom;

const int switchPins[4] = {43, 44, 45, 46};
bool switchReading1;
bool switchReading2;
bool switchReading3;
bool switchReading4;

int color;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  

  ///////////////////////////////////////////////////////////////////////
  for(int i = 0; i < 4; i++){
    pinMode(switchPins[i], INPUT);
  }

// Read the dip switch
  switchReading1 = digitalRead(switchPins[0]);
  switchReading2 = digitalRead(switchPins[1]);
  switchReading3 = digitalRead(switchPins[2]);
  switchReading4 = digitalRead(switchPins[3]);


  //Robot initialization
  atom.init();


  // ////One time tasks : Print to display  //////////////////////////////////////
  // Ramp
  if (switchReading1 && !switchReading2 && !switchReading3 && !switchReading4){
      atom.writeToDisplay(2, 10, 10, "Line");
      atom.writeToDisplay(2, 10, 30, "Following");

  } ////////////////////////// RAMP ////////////////////////////////
  else if (!switchReading1 && switchReading2 && !switchReading3 && !switchReading4){ //2
    atom.writeToDisplay(2, 10, 10, "Ramp");
    atom.ramp();

  }else if (switchReading1 && switchReading2 && !switchReading3 && !switchReading4){ //3
    atom.writeToDisplay(2, 10, 10, "Box");
    atom.writeToDisplay(2, 10, 30, "Picking");
    // atom.boxPicking();

  } else if (!switchReading1 && !switchReading2 && switchReading3 && !switchReading4){ //4

    atom.writeToDisplay(2, 10, 10, "Ball");
    atom.writeToDisplay(2, 10, 30, "Picking");
    // atom.ballPicking();

  } else if (switchReading1 && !switchReading2 && switchReading3 && !switchReading4){ //5
    atom.writeToDisplay(2, 10, 10, "Wall");
    atom.writeToDisplay(2, 10, 30, "Following");

  } else if (!switchReading1 && switchReading2 && switchReading3 && !switchReading4){ //6

    atom.writeToDisplay(1, 10, 10, "Color Detection");
    // atom.colorDetection();

  } else if (!switchReading1 && !switchReading2 && !switchReading3 && switchReading4){ //8
    atom.writeToDisplay(2, 10, 10, "Dash Line");
    atom.writeToDisplay(2, 10, 30, "Following");
    // atom.dashedLineFollow();

  }
}

void loop() {
 

  if (switchReading1 && !switchReading2 && !switchReading3 && !switchReading4){ //1
    atom.lineFollow();

  } else if (switchReading1 && switchReading2 && !switchReading3 && !switchReading4){ //3
    atom.boxPicking();

  } else if (!switchReading1 && !switchReading2 && switchReading3 && !switchReading4){ //4
    atom.ballPicking();

  } else if (switchReading1 && !switchReading2 && switchReading3 && !switchReading4){ //5
    

  } else if (!switchReading1 && switchReading2 && switchReading3 && !switchReading4){ //6

    atom.writeToDisplay(1, 10, 10, "Color Detection");
    atom.colorDetection();

  } else if (switchReading1 && switchReading2 && switchReading3 && !switchReading4){ //7
    // atom.writeToDisplay(1, 10, 10, "Object Detection");
    atom.objectDetection();

  } else if (!switchReading1 && !switchReading2 && !switchReading3 && switchReading4){ //8
    atom.dashedLineFollow();

  }
  
  
 
};
