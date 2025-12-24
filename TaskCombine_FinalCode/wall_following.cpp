#include "wall_following.h"
#include "robot.h"

WALLFollowing::WALLFollowing(Robot* robotPtr) {
  this -> robot = robotPtr; // save the same robot pointer
}

////////////////////////////////// InitWall following ////////////////////////////
void WALLFollowing::initCicrularWallFollowing(){
    sensor = 0;
    _maxWallDist = 15.0f;            // Default for outer wall..
    _minWallDist = 13.0f;

    robot -> _baseSpeed = 100;    //// 80
    robot -> _isFirstCall = false;

    robot -> turn90DegreesRight();
    robot -> _steppers.runForward(robot -> _baseSpeed, 100);


    robot -> _lastTime = millis();

    while(true){
      robot -> lineFollow();

      if (robot->tof.tofObstacleTriggered()){
        robot -> _steppers.turnRight90();

        robot -> sharpIR.read();
        distance = robot -> sharpIR.distances[sensor]; 
        robot -> _lastTime = millis();

        // robot ->clearDisplay();
        // robot ->writeToDisplay(1, 10, 10, "Triggered");
        robot ->tof.clearInterrupt();
        return;
      }
      delay(5);
    }
    



}

//////////////////////////////////////////////////////////
void WALLFollowing::initStraightWallFollowing(){
    sensor = 0;
    _maxWallDist = 25.0f;
    _minWallDist = 23.0f;

    robot -> _baseSpeed = 100;    //// 80
    robot -> _isFirstCall = false;

    // // Go forward until the wall is found
    // robot ->_steppers.setSpeed(robot ->_baseSpeed, robot -> _baseSpeed);

    // while(true){  
    //   robot -> sharpIR.readSensor(sensor);
    //   if (robot ->sharpIR.distances[0] < 30){  // Wall is found
    //     robot -> _steppers.stop();
    //     return;
    //   }

    //   robot -> _steppers.runMotors();
    // }

}

////////////////////////////////// Circular wall following /////////////////////////
void WALLFollowing::circularWallFollowing(){
        robot -> sharpIR.read();

        distance = robot -> sharpIR.distances[sensor];

        if (distance >= _minWallDist && distance <= _maxWallDist){
          //steppers.turn(baseSpeed, baseSpeed, 25);
          robot -> _steppers.setSpeed(robot -> _baseSpeed,robot -> _baseSpeed);
          robot -> _steppers.runMotors();

        }else if (distance > _maxWallDist){
          //steppers.turn(baseSpeed - 30, baseSpeed + 30, 50);}
          robot -> _steppers.setSpeed((robot -> _baseSpeed) - 30, (robot -> _baseSpeed) + 30);
          robot -> _steppers.runMotors();

        }else if (distance < _minWallDist){
        //   steppers.turn(baseSpeed + 30, baseSpeed - 30, 50);
          robot -> _steppers.setSpeed((robot -> _baseSpeed) + 50, (robot -> _baseSpeed) - 50);
          robot -> _steppers.runMotors();

        }
}

bool WALLFollowing::runStraightWallFollowing(){
  robot -> sharpIR.read();
  distance = robot -> sharpIR.distances[sensor];

  if (distance > 30) {  // Terminate case
    robot ->_steppers.stop();
    return true;
  }

  if (robot->tof.tofObstacleTriggered()){
    robot ->_steppers.turnRight90();         // 90 degree turn
    robot->tof.clearInterrupt();
  }

  if (distance >= _minWallDist && distance <= _maxWallDist){
    //steppers.turn(baseSpeed, baseSpeed, 25);
    robot -> _steppers.setSpeed(robot -> _baseSpeed,robot -> _baseSpeed);
    robot -> _steppers.runMotors();

  }else if (distance > _maxWallDist){
    //steppers.turn(baseSpeed - 30, baseSpeed + 30, 50);}
    robot -> _steppers.setSpeed((robot -> _baseSpeed) - 30, (robot -> _baseSpeed) + 30);
    robot -> _steppers.runMotors();

  }else if (distance < _minWallDist){
  //   steppers.turn(baseSpeed + 30, baseSpeed - 30, 50);
    robot -> _steppers.setSpeed((robot -> _baseSpeed) + 50, (robot -> _baseSpeed) - 50);
    robot -> _steppers.runMotors();

  }
  delay(5);
  return false;
}

bool WALLFollowing::runCircularWallFollowing(){
    if (distance > 30.0){

        if (tm_count == 0){

          tm_count++;
        
        robot -> _steppers.runForward(robot -> _baseSpeed, 300);
        robot -> _steppers.turn(-(robot -> _baseSpeed), robot -> _baseSpeed, _stepsFor90DegreeTurn);  /// Turn left
        robot -> _steppers.runForward(robot -> _baseSpeed, 600);
        robot -> _steppers.turn(-(robot -> _baseSpeed), robot -> _baseSpeed, ((_stepsFor90DegreeTurn / 2)+40));  /// Turn left
        robot -> _steppers.runForward(robot -> _baseSpeed, 300);


        if (sensor == 1) {
          sensor = 0;
          _maxWallDist = 15.0f;
          _minWallDist = 13.0f;

        }else{
        
          sensor = 1;
          _maxWallDist = 20.0f;
          _minWallDist = 15.0f;
        }



        }

        else if (tm_count == 1){

          tm_count++;
        
        robot -> _steppers.runForward(robot -> _baseSpeed, 450);
        robot -> _steppers.turn(-(robot -> _baseSpeed), robot -> _baseSpeed, _stepsFor90DegreeTurn);  /// Turn left
        robot -> _steppers.runForward(robot -> _baseSpeed, 600);
        robot -> _steppers.turn(-(robot -> _baseSpeed), robot -> _baseSpeed, ((_stepsFor90DegreeTurn / 2)+40));  /// Turn left
        robot -> _steppers.runForward(robot -> _baseSpeed, 300);


        if (sensor == 1) {
          sensor = 0;
          _maxWallDist = 15.0f;
          _minWallDist = 13.0f;

        }else{
        
          sensor = 1;
          _maxWallDist = 20.0f;
          _minWallDist = 15.0f;
        }

        

        }
     
   }

  ////////////////////////////////////////////////////////////////////////////////////////////////
  if (tm_count == 2){   // Comming back
    robot ->_irsensors.read();

    float pos = robot -> _tracker.computePosition(robot -> _irsensors.values);
 
    if (pos != 999.0f){   // line found
      robot ->_steppers.turnRight90();

      robot ->_steppers.stop();
      robot ->_lastTime = millis();
      return true;
    }
  }
  circularWallFollowing();

  return false;
}


void WALLFollowing::comeBack(){   ///// Check the logic
  bool cameBack = false;

  while(!cameBack){
    cameBack = robot ->lineFollow();
    delay(5);
  }

  // When the cross line is found
  robot -> _steppers.turnRight90();
  robot -> _steppers.runForward(robot -> _baseSpeed, 100);
  robot -> _steppers.stop();

}