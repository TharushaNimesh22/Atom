#ifndef ROBOT_H
#define ROBOT_H

//Include libraries
#include "distance_sensors.h"
#include "front_arm.h"
#include "side_arm.h"
#include "color_sensors.h"
#include "PID.h"
#include "ir_array.h"
#include "line_tracker.h"
#include "stepper_motor.h"
#include "display.h"
#include "wall_tracker.h"

//Define pins
#define FRONT_ARM_PIN 4

//LED pins
#define RED 47
#define BLUE 49
#define GREEN 48



//XHUTS pins for distance sensors are in the private attributes section

// Pins and controllers for MUX
#define MUX_ADDR 0x70  // single TCA9548A
#define MUX_DISTANCE_SENSORS 0   // MUX for distance sensors
// MUX pins for color sensors are given from a loop


//Other..
// NUM_DISTANCE_SENSORS - In distance_sensors.h
// Pins for stepper for side arm - side_arm.h - side_arm.cpp
// Other pins for individual sensor is in the coresponding library



class Robot{
  public:
      //Attributes
    //   int distanceValues[NUM_DISTANCE_SENSORS]; // Array to store the distance sensor values - Optional
      
      //Methods
      Robot();
      void init();   // Modify last time for the final evaluation
      void initPIDWallFOllowing();

      //---------------------------------SUB TASKS---------------------------------
      // void boxPicking();
      void catchBridge();
      // void ballPicking();

      void lineFollow();
      void ramp();
      void boxPicking();
      void ballPicking();
      void colorDetection();
      void objectDetection();
      void dashedLineFollow();
      void wallFollowing();
      


      ///////////////////////////////// Other Public Functions /////////////////////
      // To be removed (test case)
      // void _moveFrontArm(int pos = FRONT_ARM_INITIAL_POSITION, bool forward = false);

      int readColorSensorDisplay(int sensor, int th);

      // void _tcaSelect(uint8_t ch);

      // Dislpay functions
      void clearDisplay();
      void writeToDisplay(int textSize, int cursorX, int cursorY, char text[]);
      


  private:
      // Create instances
      FrontArm _frontArm;
      SideArm _sideArm;
      ColorSensors _colorSensors;
      DistanceSensors _distanceSensors;
      PIDController _pid;
      IRarray _irsensors;
      LINEtracker _tracker;
      STEPPERcontroller _steppers;
      Display _display;
      WALLtracker _detWall;
      

      // Attributess

      // For wheel steppers
      const float _baseSpeed = 100;      // constant forward speed
      float _steering;             // pid output
      unsigned long _lastTime;     // for fixed-time loop

      float _LOOP_DT = 0.005; // frequency of PID loop for line following

    

      // Wall Following
      const float _targetWalldistance = 60.0;  
      const float _distance_weight = 0.9;        
      const float _angle_weight = 0.7;
      const float _sensor_separation = 80.0;

      

      // Dashed line following
      bool _lineVisible;
      unsigned long _lastSeenTime;        // last time the line was detected
      const unsigned long _GAP_TIMEOUT = 1800; // ms allowed for dashed-line gaps / Cahnge according to the dashed line gap
      float _lastPos;    
      
      // Tasks navigation
      // Later
      bool _isFirstCall;
      bool _isEndOfTheTask;
      
      ///////////////// Sub tasks naviagtion  /////////////////////////////////////////
      
      // Ramp 
       bool _dashLineFollow;
       bool _completeLineLost;
       bool _crossRamp;            

      // Box picking
      bool _boxTaken;

      // To be changed...
      bool _endOfTheLine; // For box picking and line following  // Changes inside the line follow; used inside the box picking
      
      //XHUTS pins for distance sensors
      const int _xshutPins[NUM_DISTANCE_SENSORS] = {33,34,35, 36, 37, 38};


      // Methods
      // Function to select the MUX output
      void _tcaSelect(uint8_t ch);

      // Function to read the distances
      // All the sensors will be read and values will be stored in _distanceSensors.values[] array
      // Later implement - function to read only one specific sensor
      void _distanceMeasure();
      void _moveFrontArm(int pos = FRONT_ARM_INITIAL_POSITION, bool forward = false);

      // For dashed line following
      void _followDashedLine();
      void _goBackToLine();

};

#endif