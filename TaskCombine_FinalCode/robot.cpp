#include "robot.h"
#include "ramp_crossing.h"
#include "arrow_following.h"
#include "barcode.h"
#include "wall_following.h"
#include "line_follower.h"


Robot::Robot():
_lastSeenTime(0), _lastPos(0.0f),
_completeLineLost(false),   // To be changed later ///////////////////////////////////
_boxTaken(false),   // For box pickigng sub task   // Chang later
_endOfTheLine(false),  // For box picking sub task  // change later


///////////////////// For tasks navigation //////////////////
_isFirstCall(true),
_endOfTheTask(false),
tof(xshutPins, interruptPin)

{
  _rampCrossing = new RampCrossing(this);  // Pass this robot to rampcrossing
  _arrowFollowing = new ArrowFollowing(this); 
  _barcode = new BARCODEreader(this);
  _wallFollowing = new WALLFollowing(this);
  // _wallFollower = new WallFollower(this);



  //////////////Set the task navigation /////////////////////////

  for (size_t i = 0; i < 8; i++) mainTasksNavigation[i] = false;
  // mainTasksNavigation[4] = true;  // Size might be changed
}

void Robot::init(){
  //Arm
  _frontArm.init(FRONT_ARM_PIN);
  _sideArm.init();


   //Initialize the tof sensors
   // Select MUX for distance sensors
  // _tcaSelect(MUX_DISTANCE_SENSORS);
  // _distanceSensors.init(_xshutPins);
  // Initialise TOFs /////////////////////////////////////////////////////////////////////////////////
  delay(100);
  if (!tof.initAllSensors()) {
      Serial.println("TOF init failed!");
      // while (1){};
  }
  
  // Inturrupt sensor, LOW threshold, HIGH threshold
  tof.startInterruptSensor(3, 200, 0);
  delay(50);

  // Initialization of color sensors
  _colorSensors.init1(RED, GREEN, BLUE);   //DISPLAY LEDs

  // ========== Initialize 3 TCS3472 sensors ==========
  Serial.println("Initializing TCS3472 color sensors...");

  for (int i = 0; i < NUM_COLOR_SENSORS; i++) {
    _tcaSelect(i + 2); // color sensors on channels 1,2,3
    delay(50);

    _colorSensors.init2(i);

  }

  Serial.println("All sensors initialized successfully!");

  ////////////////////////////////////////////////////////////
  _irsensors.init();
  _steppers.init();
  _tracker.init();
  _detWall.init(_targetWalldistance, _distance_weight, _angle_weight, _sensor_separation);

  // Display init throuh MUX
  _tcaSelect(1);
  _display.init();

  linefollower.init();

  //Configure the PID for line following ////////////
  _pid.Kp = 55.5f;
  _pid.Ki = 0.0f;
  _pid.Kd = 0.03f;

  _pid.tau = 0.02f;

  _pid.limMax =  200;
  _pid.limMin = -200;
  _pid.limMaxInt = 50;
  _pid.limMinInt = -50;

  _pid.T  = _LOOP_DT;

  _pid.init();

  // When run the dashed line follow, add this at the begining
  _lastTime = millis();    // Modify for the final evaluation 
  // Add this to the line follow/ dash line follow an wall follow so that when the function is called for the first time, 
  // this calculation is happens and then a boolean dummy variable is changed so that when the function is called for the 
  //second time this calculation won't happen

  Serial.println("All done........");
  writeToDisplay(1, 10, 10, "All done.......");
}

// Init PID for wallfollowing
void Robot::initPIDWallFOllowing(){
  //Configure the PID
  _pid.Kp = 50.5f;
  _pid.Ki = 0.0f;
  _pid.Kd = 0.0f;

  _pid.tau = 0.02f;

  _pid.limMax =  200;
  _pid.limMin = -200;
  _pid.limMaxInt = 50;
  _pid.limMinInt = -50;

  _LOOP_DT = 0.01;
  _pid.T  = _LOOP_DT;

  _pid.init();

  _lastTime = millis();
}


// // Read the distance sensor values
// void Robot::_distanceMeasure(){
//   // Select MUX for distance sensors
//   _tcaSelect(MUX_DISTANCE_SENSORS);

//   _distanceSensors.read();
  
//   // Optional
//   // for (int i = 0; i < NUM_DISTANCE_SENSORS; i++){
//   //   distanceValues[i] = _distanceSensors.values[i];
//   // }
// }


// Function to select the MUX output
void Robot::_tcaSelect(uint8_t ch){
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
}

// Clear the display
void Robot::clearDisplay(){
  _tcaSelect(1);
  _display.clearDisplay();
}

// Write to display
void Robot::writeToDisplay(int textSize, int cursorX, int cursorY, char text[]){
  _tcaSelect(1);
  _display.write(textSize, cursorX, cursorY, text);
}
//---------------------------------SUB TASKS---------------------------------


// Move front arm forward backward
// For forward - set true; backward - false
void Robot::_moveFrontArm(int pos, bool forward){
  if (forward)_frontArm.armForward(pos);
  else _frontArm.armBackward(pos);
}


// Read the value of the given color sensor and display it using LEDs
int Robot::readColorSensorDisplay(int sensor, int th){
  _tcaSelect(sensor + 2);
  delay(50);
  return _colorSensors.readSensorDisplay(sensor, th);
}

// Function for line following //////////////////////////////////////////////////////////
bool Robot::lineFollow(){
  // Run PID loop at 10ms
  if (millis() - _lastTime >= _LOOP_DT * 1000) {

    ////////////////////////////////////
    _lastTime += _LOOP_DT * 1000;

    // Read IR sensor values
    _irsensors.read();

    // Compute the line position
    float pos = _tracker.computePosition(_irsensors.values);

    detectPerpendicularLine = _tracker.isPerpendicularLine(_irsensors.values);
    if (detectPerpendicularLine){
      // If perpendicular line is detetcted, stop the line follow
        stopTheRobot(false);  // reset complete line lost for the next task
        return true;
    }
    
    // Handle line lost( Optional )
    if (pos == 999.0f) {
        // Lost line behavior – slow turn in last known direction
        _steppers.setSpeed(+100, -100);
        _pid.integrator = 0;      // optional: prevent windup

        _endOfTheLine = true;
        return false;
    }

    // Update PID Controller
    float setpoint = 0.0f;
    float error = setpoint - pos;

    _steering = _pid.update(setpoint, pos);
    
    // Update the motor X, Y speeds
    float left  = _baseSpeed - _steering;
    float right = _baseSpeed + _steering;

    // Clamp motor commands
    if (left > 255) left = 255;
    if (left < -255) left = -255;
    if (right > 255) right = 255;
    if (right < -255) right = -255;
    
    //Run the motors
    _steppers.setSpeed(left, right);
    _steppers.runMotors();

    _endOfTheLine = false;
  }

  return false;
}


//////////////////////////////// Line Follow ends with turn ///////////////////////////
bool Robot::lineFollowEndsWithTurn(){
  // Run PID loop at 10ms
  if (millis() - _lastTime >= _LOOP_DT * 1000) {

    ////////////////////////////////////
    _lastTime += _LOOP_DT * 1000;

    // Read IR sensor values
    _irsensors.read();

    ///   Make 90 degree turn
    if (_irsensors.values[2] == 1 && _irsensors.values[3] == 1 && _irsensors.values[4] == 1){
        turn90DegreesRight();
        stopTheRobot(false);
        return true;
    }

    // Compute the line position
    float pos = _tracker.computePosition(_irsensors.values);

    detectPerpendicularLine = _tracker.isPerpendicularLine(_irsensors.values);  /// Not needed for this one
    if (detectPerpendicularLine){
      // If perpendicular line is detetcted, stop the line follow
        stopTheRobot(false);  // reset complete line lost for the next task
        return true;
    }
    
    // Handle line lost( Optional )
    if (pos == 999.0f) {
        // Lost line behavior – slow turn in last known direction
        _steppers.setSpeed(+100, -100);
        _pid.integrator = 0;      // optional: prevent windup

        _endOfTheLine = true;
        return false;
    }

    // Update PID Controller
    float setpoint = 0.0f;
    float error = setpoint - pos;

    _steering = _pid.update(setpoint, pos);
    
    // Update the motor X, Y speeds
    float left  = _baseSpeed - _steering;
    float right = _baseSpeed + _steering;

    // Clamp motor commands
    if (left > 255) left = 255;
    if (left < -255) left = -255;
    if (right > 255) right = 255;
    if (right < -255) right = -255;
    
    //Run the motors
    _steppers.setSpeed(left, right);
    _steppers.runMotors();

    _endOfTheLine = false;
  }

}


//////////////////// RAMP //////////////////////////////////////////
// void Robot::ramp(){
//     _steppers.setSpeed(_baseSpeed, _baseSpeed);    
//     delay(100);
//     _steppers.runSteps(-1500, -1500);
// }

///////////////////////// Ball Picking /////////////////////////////
void Robot::ballPicking(){
  _sideArm.moveArm(1, _sideArm.numStepsPerRev / 3.5);
  delay(1000);

  _sideArm.moveArm(-1, _sideArm.numStepsPerRev / 3.5);
  delay(2000);
}

//////////////////////// Color detection ////////////////////////
void Robot::colorDetection(){
  int color = readColorSensorDisplay(0, 2);

  // Red - 0; Green - 1, Blue - 2
  // Print to display

  switch (color){
    case 0:
      clearDisplay();
      writeToDisplay(2, 30, 30, "RED");
      break;
    case 1:
      clearDisplay();
      writeToDisplay(2, 30, 30, "GREEN");
      break;
    case 2:
      clearDisplay();
      writeToDisplay(2, 30, 30, "BLUE");
      break;

    default:
      clearDisplay();
      writeToDisplay(2, 30, 30, "MIXED");
      break;

      
  }
  delay(100);

}

/////////////////// Object Detection //////////////////
// void Robot::objectDetection(){
//   _distanceMeasure();

//   // Take the 0th and 3rd sensor values
//     int sensorBelow = _distanceSensors.values[3];
//     int sensorAbove = _distanceSensors.values[0];

//     // Serial.println(sensorBelow);
//     // Serial.println(sensorAbove);

//     char below[5];
//     char above[5];

//     sprintf(below, "%d", sensorBelow);
//     sprintf(above, "%d", sensorAbove);

//     clearDisplay();
//     writeToDisplay(1, 40, 10, above);
//     writeToDisplay(1, 40, 50, below);

//     if (sensorBelow <= 300 && sensorBelow >= 100){
//       if (sensorAbove <= 300 && sensorAbove >= 100){
//           writeToDisplay(2, 30, 30, "Obstacle");
//       }else{
//           writeToDisplay(2, 30, 30, "Object");
//       }
//     }else{
//       clearDisplay();
//     }
//     delay(10);
// }

///////////////////////// Dahsed Line Follow //////////////////////
bool Robot::dashedLineFollow(int firstTurningDirection){

  // First Turning Direction: First turning direction if the dashed line is missed
    // Left: -1, right: +1

    if (!_completeLineLost){
      // If the dash line is available, follow the dash line
        return _followDashedLine();
    }else{
      // If the line is missed, go back until the line is detected again and then turn left
        return _goBackToLine(firstTurningDirection);
    }
}

///////////////////////////////////////////////
// Function for line following
bool Robot::_followDashedLine(){
    // Run PID loop at 10ms
  if (millis() - _lastTime >= _LOOP_DT * 1000) {
    _lastTime += _LOOP_DT * 1000;

    // Read IR sensor values
    _irsensors.read();


    detectPerpendicularLine = _tracker.isPerpendicularLine(_irsensors.values);
    if (detectPerpendicularLine){
      // If perpendicular line is detetcted, stop the line follow
        ////////////////
        alignWithPerpendicularLine();
        delay(100);
        stopTheRobot(false);  // reset complete line lost for the next task
        return true;
    }
    // Compute the line position
    float pos = _tracker.computePosition(_irsensors.values);
    
    // If line is found
    if (pos != 999.0f) {
        _lineVisible = true;
        _lastSeenTime = millis();
        _lastPos = pos;
    }
    // If line is not found
    else {
        _lineVisible = false;
    }

    // Handle dashed line
    if (!_lineVisible && (millis() - _lastSeenTime < _GAP_TIMEOUT)) {
        // Continue with the previous ir tracks
        pos = _lastPos;
    }

    // Handle complete line lost
    if (!_lineVisible && (millis() - _lastSeenTime >= _GAP_TIMEOUT)) {
        // Line lost fallback
        _steppers.setSpeed(+100, -100);
        _pid.integrator = 0;
        _completeLineLost = true;
        delay(1000);
        return false;
    }

    // Update PID Controller
    float setpoint = 0.0f;
    float error = setpoint - pos;

    _steering = _pid.update(setpoint, pos);
    
    // Update the motor X, Y speeds
    float right = _baseSpeed + _steering;
    float left = _baseSpeed - _steering;

    // Clamp motor commands
    if (left > 255) left = 255;
    if (left < -255) left = -255;
    if (right > 255) right = 255;
    if (right < -255) right = -255;
    
    //Run the motors
    _steppers.setSpeed(left, right);
    _steppers.runMotors();
  }

  return false;
}

/////////////////////////////////////////////////
// Function for go back to the dash line
bool Robot::_goBackToLine(int firstTurningDirection){
    bool lineFound = false;
    // Read IR sensor values
    _irsensors.read();

    // Compute the line position
    float pos = _tracker.computePosition(_irsensors.values);

    if (pos != 999.0f) {
      lineFound = true;
    }

    // If line is found
    if (lineFound) {
      // delay(1000);
      // turn left and move forward
      float right = -(firstTurningDirection * _baseSpeed);   // BaseSpeed 100, 80 are preffered
      float left = firstTurningDirection * _baseSpeed;
      // _steppers.setSpeed(left, right);

      _steppers.turn(left, right, 70);           // Initial 80
      delay(30);

        if (pos != 999.0f){
            _completeLineLost = false;
            _lastPos = 0.0f;
            _lastSeenTime = millis();
        }else{
          _steppers.setSpeed(_baseSpeed, _baseSpeed);
          _steppers.runMotors();
          delay(5);
        }
       

    }else{
      //come backwards until the line is detected
      //Run the motors
      _steppers.setSpeed(-_baseSpeed, -_baseSpeed);
      _steppers.runMotors();
      delay(5);
    }

    return false;
}

////////////////////////////////////////// Upgraded version ////////////////////////////
bool Robot::_goBackToLine2(int firstTurningDirection){

  bool lineFound = false;
  // // Read IR sensor values
    _irsensors.read();

    // Compute the line position
    float pos = _tracker.computePosition(_irsensors.values);

    if (pos != 999.0f) {
      lineFound = true;
    }

    if (lineFound){
      //// Check both directions
        // turn left and move forward
      float right = (-1)*(firstTurningDirection * _baseSpeed);
      float left = firstTurningDirection * _baseSpeed;
      // _steppers.setSpeed(left, right);

      _steppers.turn(left, right, 40);
      delay(30);
      
      _steppers.setSpeed(_baseSpeed, _baseSpeed);
      _lastSeenTime = millis();

      // bool nextLineFound = false;
      /// GO forward in the first direction and look for the line
      while (millis() - _lastSeenTime < _GAP_TIMEOUT){
        _irsensors.read();
        pos = _tracker.computePosition(_irsensors.values);

        if (pos != 999.0f){
            _lastSeenTime = millis();
            return false;
        }
        _steppers.runMotors();
        delay(5);

      }

      /////////////////// If the next line is not found at the first turn, come back ///////////////
      _steppers.setSpeed(-_baseSpeed, -_baseSpeed);
      while(pos == 999.0f){
          _steppers.runMotors();

          _irsensors.read();
          pos = _tracker.computePosition(_irsensors.values);
      }

      /////////////////////////// Then turn around and search the other direction //////////////////
      right = firstTurningDirection * _baseSpeed;
      left = (-1)*(firstTurningDirection * _baseSpeed);
      // _steppers.setSpeed(left, right);

      _steppers.turn(left, right, 80);
      delay(30);
      
      
      _steppers.setSpeed(_baseSpeed, _baseSpeed);
     

      /// Run little bit forward to go out of the current line
      while (pos != 999.0f){
        _irsensors.read();
        pos = _tracker.computePosition(_irsensors.values);
        _steppers.runMotors();
        delay(5);
      }

       _lastSeenTime = millis();

      // bool nextLineFound = false;
      while (millis() - _lastSeenTime < _GAP_TIMEOUT){
        _irsensors.read();
        pos = _tracker.computePosition(_irsensors.values);

        if (pos != 999.0f){
            _lastSeenTime = millis();
            return false;
        }
        _steppers.runMotors();
        delay(5);

      }

      while(true){
        _steppers.stop();
      }
    }else{
      //come backwards until the line is detected
      //Run the motors
      _steppers.setSpeed(-_baseSpeed, -_baseSpeed);
      _steppers.runMotors();
      delay(5);
    }


    return false;
}

/////////////////////////////// Box Picking /////////////////////////////////
// void Robot::boxPicking(){
//   // If a box is not taken, look for a box
//   if (!_boxTaken){
//     // Measure the distance (Take the average distance)
//     float avgDistance = 0.0f;

//     for (int i = 0; i < 10; i++){
//         _distanceMeasure();
//         avgDistance += _distanceSensors.values[3];
//     }
    
//     avgDistance /= 10;
    
//     char avg[10];
//     sprintf(avg, "%f", avgDistance);

//     writeToDisplay(1, 10, 50, avg);

//     // If there is a box take it
//     if (avgDistance > 0.0 && avgDistance <= 40.0){
//       _frontArm.initialPosition();
//       _frontArm.catchBox();
//       _boxTaken = true;

//       delay(500);
//     }

//   }else{
//     // If a box is taken, follow the line and  release box
//     lineFollow();

//     if (_endOfTheLine){ // Reach the end of the line
//       _frontArm.releaseBox();  // Release box
//       _boxTaken = false;
//       delay(5000);
//     }

//   }
// }

///////////////////////// Wall Following ///////////////////////////////////
// void Robot::wallFollowing(){
//     // Run PID loop at desired frequency
//   if (millis() - _lastTime >= _LOOP_DT * 1000) {
//     _lastTime += _LOOP_DT * 1000;

//     // Get _steering correction from wall follower module
//     _distanceMeasure();


//     float  combinedVertualMeasurement = _detWall.computeCombinedVertualMeasurement(_distanceSensors.values);
//     _steering = _pid.update(0, combinedVertualMeasurement);

//     // Convert to motor speeds
//     float left  = _baseSpeed - _steering;
//     float right = _baseSpeed + _steering;

//     // Clamp motor commands
//     if (left > 255) left = 255;
//     if (left < -255) left = -255;
//     if (right > 255) right = 255;
//     if (right < -255) right = -255;

//     //Run the motors
//     _steppers.setSpeed(left, right);
//     _steppers.runMotors();
//   }
// }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Some of motor control functions 

//////////////////////////////////////// Stop the robot /////////////////////
void Robot::stopTheRobot(bool completelineLost){
      _pid.integrator = 0;
      _steppers.setSpeed(+100, -100);
      _completeLineLost = completelineLost;
}

///////////////////////////////  turn back/////////////////////
void Robot::turnBack(){
  _steppers.turn(_baseSpeed, -_baseSpeed, _stepesForTurnBack);  // Base speed 100 is preffered
}

void Robot::turn90DegreesRight(){
  // 1 = turn right; -1 = turn left  Default: Turn right

  // float speed = _baseSpeed * dire
  _steppers.turn(_baseSpeed, -_baseSpeed, _stepsFor90DegreeTurn);

  // Base speed 80 is preferred

}

void Robot::turn90DegreesLeft(){
  // 1 = turn right; -1 = turn left  Default: Turn right

  // float speed = _baseSpeed * dire
  _steppers.turn(-_baseSpeed, _baseSpeed, _stepsFor90DegreeTurn);
}

/////////////////////////////////////////////////////////////////

//////////////// Sharp turn ///////////////////////////////////
void Robot::sharpTurn(int direction){
  
      _steppers.runForward(_baseSpeed, 100);
      // turn90DegreesLeft();     // Later implement (make a big turn for one senosr detection)

      float left = direction * _baseSpeed;
      float right = -(direction * _baseSpeed);

      _steppers.turn(left, right, 120);          //160
      // _steppers.runForward(-_baseSpeed, 200);      ///////////  REmove
      delay(10);

      // After the turn check if the line is detected as a perpendicular line
      _irsensors.read();
      detectPerpendicularLine = _tracker.isPerpendicularLine(_irsensors.values);
      if (detectPerpendicularLine){
          _steppers.turn(left, right, 50);   // Change this as needed
          detectPerpendicularLine = false;
      }
      //////////////////////////////////

      _lineVisible = true;
      _lastSeenTime = millis();
      ///////////////////////////////////////////////////////////////////////////////
}

/////////////////////////////// 
// Function to check weather all the tasks are done 
// If done: returns true, false otherwise
bool Robot::allTasksDone(bool *tasksNavigation, size_t length){
  // tasksNavigation: task navigation array, length: length of the array

  for (size_t i = 0; i < length; i++){
    if (tasksNavigation[i]) return false;
  }

  return true;
}

///////////////////////////////////////////////////////////
////////////// When a task is done, get ready for the next ///////////////

void Robot::_taskDone(int task){
  mainTasksNavigation[task] = false;
        // mainTasksNavigation[5] = true;
      
      if ((task + 1) < _numMainTasks) mainTasksNavigation[task + 1] = true;

      _endOfTheTask = false;  // Reset for the next task
      _isFirstCall = true;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Main tasks ////////////////////////////////////////////////
void Robot::startCompetition(){
      if (mainTasksNavigation[0]){  // grid solving
          runMainControlLoop();
  
      } else if (mainTasksNavigation[1]){    // Ramp crossing /////////////////////////////

          if (_isFirstCall) _rampCrossing -> initRampCrossing();          /// Set the parameters for ramp crossing task

          _rampCrossing -> rampCrossing();
          ///////////////////// Check the end of the main task //////////////
          _endOfTheTask = allTasksDone(_rampCrossing -> subTasksNavigation, 4);

          if (_endOfTheTask){  // When the task is done
            _taskDone(1);

           
          }


    ////////////////////////////////////////////////////////////////////////////////////////////////
      } else if(mainTasksNavigation[2]){  ////////////////// Barcode Reading ///////////////////////

          if (_isFirstCall) {
             /// 
            clearDisplay();
            writeToDisplay(1, 10, 10, "Barcode started");
            _barcode -> init();
            
            }

          _endOfTheTask = _barcode -> run();

          if (_endOfTheTask) {  // When the task is done
              _taskDone(2);
          }

      ///////////////////////////////////////////////////////////////////////////////////////
    }  else if(mainTasksNavigation[3]){  ////////////////// Circular wall following and ball picking ///////////////////////

          if (_isFirstCall) _wallFollowing -> initCicrularWallFollowing();

          _endOfTheTask = _wallFollowing -> runCircularWallFollowing();

          if (_endOfTheTask) {  // When the task is done
            _wallFollowing -> comeBack();
             _taskDone(3);
            
            // stopTheRobot(false);  //////////// Change this
            // _steppers.stop();
          }

    } else if (mainTasksNavigation[4]){ //// arrow following ////////////////////////////////

          if (_isFirstCall) _arrowFollowing -> initArrowFollowing();  // Initializa the task parameters


          _endOfTheTask = _arrowFollowing -> arrowFollowing();   // Do the task

          if (_endOfTheTask){  // When the task is done
              _taskDone(4);
            }


      } else if (mainTasksNavigation[5]){ //// straight wall following ////////////////////////////////

          if (_isFirstCall) _wallFollowing -> initStraightWallFollowing();  // Initializa the task parameters


          _endOfTheTask = _wallFollowing -> runStraightWallFollowing();   // Do the task

          if (_endOfTheTask){  // When the task is done
              _taskDone(5);
            }
      }
};


////////////////////////////////////////////////////////////////////////////////////////
void Robot::alignWithPerpendicularLine(){
  bool aligned;
  float pos;
  float tempPerpendicularLine;
  int count = 0;

  while (detectPerpendicularLine){
        _irsensors.read();
        pos = _tracker.computePosition(_irsensors.values);
        tempPerpendicularLine = _tracker.isPerpendicularLine(_irsensors.values);

        if (pos == 999.0f){
          stopTheRobot(false);
          detectPerpendicularLine = false;
          return;
        } else if (tempPerpendicularLine){
          _steppers.setSpeed(80, 80);
          _steppers.runMotors();
          delay(5);

        } else if (_irsensors.values[0] == 0){
          _steppers.setSpeed(-80, 0);
          _steppers.runMotors();
          delay(100);

           count++;

          if (count == 50) {
            detectPerpendicularLine = false;
            return;
          }
        } else if (_irsensors.values[4] == 0){
          _steppers.setSpeed(0, -80);
          _steppers.runMotors();
          delay(100);

           count++;

          if (count == 50) {
            detectPerpendicularLine = false;
            return;
          }
        }

  }
}
///////////////////////////////////////////////////////////// GRID //////////////////////////////////////////////////////////////////
void Robot::handleObstacle(){


  // Stop the robot
  _steppers.stop();
  _steppers.runMotors();
  delay(500);

  // --- Find the previous junction ---
  _irsensors.read();
  bool isJunctionEdge = _tracker.detectJunction(_irsensors.values);

  while (!isJunctionEdge) {
    linefollower.update(false);
    _irsensors.read();
    isJunctionEdge = _tracker.detectJunction(_irsensors.values);
  }

  delay(500);

  // -------------------------
  //   OBSTACLE → EAST CASE
  // -------------------------
  if (heading == EAST) {

    // Shift forward slightly
    for (int i = 0; i < 35; i++) {
      _steppers.setSpeed(200, 200);
      _steppers.runMotors();
      delay(10);
    }

    _steppers.turnLeft90();

    // Reach next junction
    _irsensors.read();
    isJunctionEdge = _tracker.detectJunction(_irsensors.values);

    while (!isJunctionEdge) {
      linefollower.update(true);
      _irsensors.read();
      isJunctionEdge = _tracker.detectJunction(_irsensors.values);
    }

    delay(500);

    for (int i = 0; i < 35; i++) {
      _steppers.setSpeed(200, 200);
      _steppers.runMotors();
      delay(10);
    }

    _steppers.turnRight90();
    delay(500);

    // Continue to next junction
    _irsensors.read();
    isJunctionEdge = _tracker.detectJunction(_irsensors.values);
    while (!isJunctionEdge) {
      linefollower.update(true);
      _irsensors.read();
      isJunctionEdge = _tracker.detectJunction(_irsensors.values);
    }

    // Second junction adjustment
    for (int i = 0; i < 50; i++) {
      _steppers.setSpeed(200, 200);
      _steppers.runMotors();
      delay(10);
    }

    _irsensors.read();
    isJunctionEdge = _tracker.detectJunction(_irsensors.values);
    while (!isJunctionEdge) {
      linefollower.update(true);
      _irsensors.read();
      isJunctionEdge = _tracker.detectJunction(_irsensors.values);
    }

    delay(500);

    // Forward slightly
    for (int i = 0; i < 35; i++) {
      _steppers.setSpeed(200, 200);
      _steppers.runMotors();
      delay(10);
    }

    _steppers.turnRight90();
    delay(500);

    // Final alignment
    for (int i = 0; i < 35; i++) {
      _steppers.setSpeed(200, 200);
      _steppers.runMotors();
      delay(10);
    }

    _steppers.turnLeft90();
    delay(500);

    for (int i = 0; i < 35; i++) {
      _steppers.setSpeed(200, 200);
      _steppers.runMotors();
      delay(10);
    }

    // SHIFT GRID
    col += 2;
  }

  // -------------------------
  //   OBSTACLE → WEST CASE
  // -------------------------
  else if (heading == WEST) {
    _steppers.turnRight90();
    _lastTime = millis();

    while (!_tracker.detectJunction(_irsensors.values)) {
      linefollower.update(true);
    }

    _steppers.turnLeft90();
    _lastTime = millis();

    while (_tracker.detectJunction(_irsensors.values)) {
      linefollower.update(true);
    }

    for (int i = 0; i < 11; i++) {
      _steppers.runMotors();
      delay(5);
    }

    _lastTime = millis();

    while (_tracker.detectJunction(_irsensors.values)) {
      linefollower.update(true);
    }

    _steppers.turnLeft90();
    _lastTime = millis();

    while (!_tracker.detectJunction(_irsensors.values)) {
      linefollower.update(true);
    }

    _steppers.turnRight90();

    // SHIFT GRID
    col -= 2;
  }

  // Clear TOF interrupt
  tof.clearInterrupt();
  previouslyTurntaken = true;
  _lastTime = millis();

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Robot::updateGridPosition() {
  switch (heading) {
    case NORTH: row++; break;
    case SOUTH: row--; break;
    case EAST:  col++; break;
    case WEST:  col--; break;
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Robot::runMainControlLoop() {

  // Run at 10ms period
  if (millis() - _lastTime < _LOOP_DT * 1000) return;
  _lastTime += _LOOP_DT * 1000;

  // ------------------ IR + PID ------------------
  _irsensors.read();
  float pos = _tracker.computePosition(_irsensors.values);

  if (pos == 999.0f) {
      _steppers.setSpeed(+100, -100);
      _pid.integrator = 0;
      return;
  }

  _steering = _pid.update(0.0f, pos);

  float right = _baseSpeed + _steering;
  float left  = _baseSpeed - _steering;

  left  = constrain(left,  -255, 255);
  right = constrain(right, -255, 255);

  _steppers.setSpeed(left, right);
  _steppers.runMotors();

  // ------------------ JUNCTIONS ------------------
  bool isJunction = _tracker.detectJunction(_irsensors.values);

  if (isJunction && !previouslyAtjunction) {
      updateGridPosition();
      previouslyAtjunction = true;
      previouslyTurntaken = false;
  }

  if (!isJunction) {
      previouslyAtjunction = false;
  }

  // ------------------ SNAKE TURNS ------------------
  if (( col == 8 ) && ( row % 2 == 0 ) && !previouslyTurntaken) {
      heading = NORTH;
      _steppers.turnLeft90();
      previouslyTurntaken = true;
      _lastTime = millis();
      return;
  }

  if (( col == 8 ) && ( row % 2 == 1 ) && !previouslyTurntaken) {
      heading = WEST;
      _steppers.turnLeft90();
      previouslyTurntaken = true;
      _lastTime = millis();
      return;
  }

  if (( col == 0 ) && ( row % 2 == 1 ) && !previouslyTurntaken) {
      heading = NORTH;
      _steppers.turnRight90();
      previouslyTurntaken = true;
      _lastTime = millis();
      return;
  }

  if (( col == 0 ) && ( row % 2 == 0 ) && (row != 0) && !previouslyTurntaken) {
      heading = EAST;
      _steppers.turnRight90();
      previouslyTurntaken = true;
      _lastTime = millis();
      return;
  }

  // ------------------ END CONDITION ------------------
  if (( col == 0 ) && (row == 7 ) && !previouslyTurntaken) {
      _steppers.setSpeed(0, 0);
      _pid.integrator = 0;
      while (true) {}
  }

  // ------------------ OBSTACLE ------------------
  if (tof.tofObstacleTriggered()) {
      handleObstacle();

      tof.clearInterrupt();
      return;
  }
}

