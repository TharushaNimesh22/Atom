#ifndef ROBOT_H
#define ROBOT_H

//Include libraries
// #include "distance_sensors.h"
#include "front_arm.h"
#include "side_arm.h"
#include "color_sensors.h"
#include "PID.h"
#include "ir_array.h"
#include "line_tracker.h"
#include "stepper_motor.h"
#include "display.h"
#include "wall_tracker.h"
#include "tof_manager.h"
#include "sharp_ir.h"
#include "line_follower.h"

//Define pins
#define FRONT_ARM_PIN 4

//LED pins
#define RED 47
#define BLUE 49
#define GREEN 48



//XHUTS pins for distance sensors are in the private attributes section

// Pins and controllers for MUX
#define MUX_ADDR 0x70  // single TCA9548A
// #define MUX_DISTANCE_SENSORS 0   // MUX for distance sensors
// MUX pins for color sensors are given from a loop


//Other..
// NUM_DISTANCE_SENSORS - In distance_sensors.h
// Pins for stepper for side arm - side_arm.h - side_arm.cpp
// Other pins for individual sensor is in the coresponding library\\





class RampCrossing;
class ArrowFollowing;
class BARCODEreader;
class WALLFollowing;


class Robot{
  public:
      Robot();

      //XHUTS pins for distance sensors
      uint8_t xshutPins[6] = {33, 34,35, 36, 37, 38};
      const uint8_t interruptPin = 19;  // interrupt Pin

    // Create instances
      FrontArm _frontArm;
      SideArm _sideArm;
      ColorSensors _colorSensors;
      // DistanceSensors _distanceSensors;
      PIDController _pid;
      IRarray _irsensors;
      LINEtracker _tracker;
      STEPPERcontroller _steppers;
      Display _display;
      WALLtracker _detWall;
      TOF_Manager tof;
      SharpIR sharpIR;
      LineFollower linefollower;

            // GRID
      bool grid[8][9];

      // Starting grid position
      int row = 0;
      int col = 0;

      enum Direction { NORTH, EAST, SOUTH, WEST };

      // Starting Direction
      Direction heading = EAST;

      // Avoid junction debounce
      bool previouslyAtjunction = false;

      // Avoid turning debounce
      bool previouslyTurntaken;

      // // TOF pins
      // uint8_t xshutPins[6] = {33, 34, 35, 36, 37, 38};
      // const uint8_t interruptPin = 19;
      

      // Attributess
      bool mainTasksNavigation[8];     // Size might be changed
      //                       = {false,  // grid solving   0
      //                         false,  // ramp crossing   1
      //                         false,  // barcode reading 2
      //                         false,  // ballpicking     3
      //                         true,   // arrow following 4


      // };
      const int _numMainTasks = 5;

      // For wheel steppers
      float _baseSpeed = 100;      // constant forward speed
      float _steering;             // pid output
      unsigned long _lastTime;     // for fixed-time loop

      float _LOOP_DT = 0.005; // frequency of PID loop for line following

      const int _stepsFor90DegreeTurn = 240;
      const int _stepesForTurnBack = _stepsFor90DegreeTurn * 2;
    

      // Wall Following
      const float _targetWalldistance = 60.0;  
      const float _distance_weight = 0.9;        
      const float _angle_weight = 0.7;
      const float _sensor_separation = 160.0;

      

      // Dashed line following
      bool _lineVisible;
      unsigned long _lastSeenTime;        // last time the line was detected
      unsigned long _GAP_TIMEOUT = 1800; // ms allowed for dashed-line gaps / Cahnge according to the dashed line gap
      float _lastPos;    

      ////////////////////////////////////////////////////////////////
      int barcodeOutput;
      
      // Tasks navigation
      // Later
      bool _isFirstCall;
      bool _endOfTheTask;
      // bool _isEndOfTheTask;
      bool detectPerpendicularLine;
      
      ///////////////// Sub tasks naviagtion  /////////////////////////////////////////
      
      // Ramp 
       bool _dashLineFollow;
       bool _completeLineLost;
       bool _crossRamp;            

      // Box picking
      bool _boxTaken;

      // To be changed...
      bool _endOfTheLine; // For box picking and line following  // Changes inside the line follow; used inside the box picking
      
      
      //Methods :: Grid
      void handleObstacle();
      void updateGridPosition();
      void runMainControlLoop();

      void init();   // Modify last time for the final evaluation
      void initPIDWallFOllowing();

      ////////////////////////////////// Main function /////////////////////////////
      void startCompetition();       /// Implement

      ///////////////////////////////////////////////////
      //---------------------------------SUB TASKS---------------------------------
      // void catchBridge();   /// not needed right now

      bool lineFollow();   
      bool lineFollowEndsWithTurn();
      // void ramp();         /// not needed right now
      // void boxPicking();   /// not needed right now
      void ballPicking();  /// not needed right now
      void colorDetection(); /// not needed right now
      // void objectDetection(); /// not needed right now

      bool dashedLineFollow(int firstTurningDirection);         /// Needed
      // First Turning Direction: First turning direction if the dashed line is missed
    // Left: -1, right: +1

      bool _goBackToLine(int firstTurningDirection);
      bool _goBackToLine2(int firstTurningDirection);

      
      ///////////////////////////////// Other Public Functions /////////////////////
      // To be removed (test case)
      // void _moveFrontArm(int pos = FRONT_ARM_INITIAL_POSITION, bool forward = false);

      int readColorSensorDisplay(int sensor, int th);

      // void _tcaSelect(uint8_t ch);

      // Dislpay functions
      void clearDisplay();
      void writeToDisplay(int textSize, int cursorX, int cursorY, char text[]);

      bool allTasksDone(bool *tasksNavigation, size_t length);

      //////////////////////////////// For init methods ///////////////////////////////////
      // robot motion control functions
      void stopTheRobot(bool completelineLost);   /// Might be able make this private
      void turnBack();
      void turn90DegreesRight();
      void turn90DegreesLeft();
      void sharpTurn(int direction);
      void alignWithPerpendicularLine();


  private:
      RampCrossing* _rampCrossing;                 ///  Rampcrossing main task
      ArrowFollowing* _arrowFollowing;
      BARCODEreader* _barcode;
      WALLFollowing* _wallFollowing;
      
      
      

    

      

      // ////////////////////////////////////////
      // /// parameters for circular wall following
      // const float _maxCircularWallDist = 18.0f;
      // const float _minCircularWallDist = 15.0f;

      // Methods
      // Function to select the MUX output
      void _tcaSelect(uint8_t ch);

      // Function to read the distances
      // All the sensors will be read and values will be stored in _distanceSensors.values[] array
      // Later implement - function to read only one specific sensor
      // void _distanceMeasure();
      void _moveFrontArm(int pos = FRONT_ARM_INITIAL_POSITION, bool forward = false);

      // For dashed line following
      bool _followDashedLine();

      //////////////////////////// Main Tasks /////////////////////////////////////////////////////
      void _taskDone(int task);
};

#endif