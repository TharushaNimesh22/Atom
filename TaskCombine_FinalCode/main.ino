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


  atom.clearDisplay();
  // ////One time tasks : Print to display  //////////////////////////////////////
  // Ramp
  if (switchReading1 && !switchReading2 && !switchReading3 && !switchReading4){     //1
      atom.mainTasksNavigation[0] = true;
      atom.writeToDisplay(2, 10, 10, "Grid");
      atom.writeToDisplay(2, 10, 30, "Solving");

  } ////////////////////////// RAMP ////////////////////////////////
  else if (!switchReading1 && switchReading2 && !switchReading3 && !switchReading4){ //2
    atom.mainTasksNavigation[1] = true;
    atom.writeToDisplay(2, 10, 10, "Ramp");
    // atom.ramp();

  }else if (switchReading1 && switchReading2 && !switchReading3 && !switchReading4){ //3
    atom.mainTasksNavigation[2] = true;
    atom.writeToDisplay(2, 10, 10, "Barcode");
    // atom.writeToDisplay(2, 10, 30, "Picking");
    // atom.boxPicking();

  } else if (!switchReading1 && !switchReading2 && switchReading3 && !switchReading4){ //4
    atom.mainTasksNavigation[3] = true;
    atom.writeToDisplay(2, 10, 10, "Ball");
    atom.writeToDisplay(2, 10, 30, "Picking");
    // atom.ballPicking();

  } else if (switchReading1 && !switchReading2 && switchReading3 && !switchReading4){ //5
    atom.mainTasksNavigation[4] = true;
    atom.writeToDisplay(2, 10, 10, "Arrow");
    atom.writeToDisplay(2, 10, 30, "Following");

  } else if (!switchReading1 && switchReading2 && switchReading3 && !switchReading4){ //6
    atom.mainTasksNavigation[5] = true;

    atom.writeToDisplay(2, 10, 10, "Wall");
    atom.writeToDisplay(2, 10, 30, "Following");
    // atom.colorDetection();

  } else if (switchReading1 && switchReading2 && switchReading3 && !switchReading4){ //7
    atom.mainTasksNavigation[6] = true;
    atom.writeToDisplay(2, 10, 10, "Hidden");
    atom.writeToDisplay(2, 10, 30, "Task");
    // atom.dashedLineFollow();

  } else if (!switchReading1 && !switchReading2 && !switchReading3 && switchReading4){ //8
    atom.mainTasksNavigation[7] = true;
    atom.writeToDisplay(2, 10, 10, "Ball");
    atom.writeToDisplay(2, 10, 30, "Shooting");
    // atom.dashedLineFollow();

  }
}

void loop() {
 
  // atom.startCompetition();
  atom.startCompetition();
   
 
};
