#include "robot.h"

Robot::Robot():
_lastSeenTime(0), _lastPos(0.0f),
_completeLineLost(false),   // To be changed later ///////////////////////////////////
_boxTaken(false),   // For box pickigng sub task   // Chang later
_endOfTheLine(false),  // For box picking sub task  // change later


///////////////////// For tasks navigation //////////////////
_isFirstCall(true),
_isEndOfTheTask(false)
{
}

void Robot::init(){
  //Arm
  _frontArm.init(FRONT_ARM_PIN);
  _sideArm.init();


   //Initialize the distance sensors
   // Select MUX for distance sensors
  _tcaSelect(MUX_DISTANCE_SENSORS);
  _distanceSensors.init(_xshutPins);

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


// Read the distance sensor values
void Robot::_distanceMeasure(){
  // Select MUX for distance sensors
  _tcaSelect(MUX_DISTANCE_SENSORS);

  _distanceSensors.read();
  
  // Optional
  // for (int i = 0; i < NUM_DISTANCE_SENSORS; i++){
  //   distanceValues[i] = _distanceSensors.values[i];
  // }
}


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


void Robot::catchBridge(){
    

}
// // Ball Picking
// void Robot::ballPicking(){
//   ////////////////////// Need to implement /////////////////////////////
//   _sideArm.pickBall();
// }


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
void Robot::lineFollow(){
  // Run PID loop at 10ms
  if (millis() - _lastTime >= _LOOP_DT * 1000) {

    ////////////////////////////////////
    _lastTime += _LOOP_DT * 1000;

    // Read IR sensor values
    _irsensors.read();

    // Compute the line position
    float pos = _tracker.computePosition(_irsensors.values);
    
    // Handle line lost( Optional )
    if (pos == 999.0f) {
        // Lost line behavior – slow turn in last known direction
        _steppers.setSpeed(+100, -100);
        _pid.integrator = 0;      // optional: prevent windup

        _endOfTheLine = true;
        return;
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
void Robot::ramp(){
    _steppers.setSpeed(_baseSpeed, _baseSpeed);    
    delay(100);
    _steppers.runSteps(-1500, -1500);

  //  moveMotors(-1200, -1200);
  // delay(1000);
  // moveMotors(0, 0);
  // delay(1000);

}

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
void Robot::objectDetection(){
  _distanceMeasure();

  // Take the 0th and 3rd sensor values
    int sensorBelow = _distanceSensors.values[3];
    int sensorAbove = _distanceSensors.values[0];

    // Serial.println(sensorBelow);
    // Serial.println(sensorAbove);

    char below[5];
    char above[5];

    sprintf(below, "%d", sensorBelow);
    sprintf(above, "%d", sensorAbove);

    clearDisplay();
    writeToDisplay(1, 40, 10, above);
    writeToDisplay(1, 40, 50, below);

    if (sensorBelow <= 300 && sensorBelow >= 100){
      if (sensorAbove <= 300 && sensorAbove >= 100){
          writeToDisplay(2, 30, 30, "Obstacle");
      }else{
          writeToDisplay(2, 30, 30, "Object");
      }
    }else{
      clearDisplay();
    }
    delay(10);
}

///////////////////////// Dahsed Line Follow //////////////////////
void Robot::dashedLineFollow(){
    if (!_completeLineLost){
      // If the dash line is available, follow the dash line
        _followDashedLine();
    }else{
      // If the line is missed, go back until the line is detected again and then turn left
        _goBackToLine();
    }
}

///////////////////////////////////////////////
// Function for line following
void Robot::_followDashedLine(){
    // Run PID loop at 10ms
  if (millis() - _lastTime >= _LOOP_DT * 1000) {
    _lastTime += _LOOP_DT * 1000;

    // Read IR sensor values
    _irsensors.read();


    bool detectPerpendicularLine = _tracker.isPerpendicularLine(_irsensors.values);
    if (detectPerpendicularLine){
      // If perpendicular line is detetcted, stop the line follow
        _pid.integrator = 0;
        _steppers.setSpeed(+100, -100);
        return;
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
        return;
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
}

/////////////////////////////////////////////////
// Function for go back to the dash line
void Robot::_goBackToLine(){
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
      float right = _baseSpeed + 40;
      float left = _baseSpeed - 40;
      // steppers.setSpeed(left, right);

      _steppers.turn(left, right, 200);

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
}

/////////////////////////////// Box Picking /////////////
void Robot::boxPicking(){
  // If a box is not taken, look for a box
  if (!_boxTaken){
    // Measure the distance (Take the average distance)
    float avgDistance = 0.0f;

    for (int i = 0; i < 10; i++){
        _distanceMeasure();
        avgDistance += _distanceSensors.values[3];
    }
    
    avgDistance /= 10;
    
    char avg[10];
    sprintf(avg, "%f", avgDistance);

    writeToDisplay(1, 10, 50, avg);

    // If there is a box take it
    if (avgDistance > 0.0 && avgDistance <= 40.0){
      _frontArm.initialPosition();
      _frontArm.catchBox();
      _boxTaken = true;

      delay(500);
    }

  }else{
    // If a box is taken, follow the line and  release box
    lineFollow();

    if (_endOfTheLine){ // Reach the end of the line
      _frontArm.releaseBox();  // Release box
      _boxTaken = false;
      delay(5000);
    }

  }
}

///////////////////////// Wall Following ///////////////////////////////////
void Robot::wallFollowing(){
    // Run PID loop at desired frequency
  if (millis() - _lastTime >= _LOOP_DT * 1000) {
    _lastTime += _LOOP_DT * 1000;

    // Get steering correction from wall follower module
    _distanceMeasure();


    float  combinedVertualMeasurement = _detWall.computeCombinedVertualMeasurement(_distanceSensors.values);
    _steering = _pid.update(0, combinedVertualMeasurement);

    // Convert to motor speeds
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
  }
}

// 