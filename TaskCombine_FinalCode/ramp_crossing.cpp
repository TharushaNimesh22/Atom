#include "ramp_crossing.h"
#include "robot.h"

RampCrossing::RampCrossing(Robot* robotPtr) {
  this -> robot = robotPtr; // save the same robot pointer
}

///////////////////////////
void RampCrossing::initRampCrossing(){
  // subTasksNavigation = new bool[4];

  // ubTasksNavigation[] = {true, // Initial dash line follow
  //                           false, // Align the robot with the perpendicular line, Make the bridge, 
  //                                   //come back to perpendicular line and turn
  //                           false, // Follow the dash line and reach the begining of the task
  //                           false, // Come back little bit, turn 90 degrees and cross the ramp,
  //                                 //  Stop when reach the perpendicular line

 
  for (int i = 0; i < 4; i++) subTasksNavigation[i] = false;
 subTasksNavigation[0] = true;
  robot -> detectPerpendicularLine = false;

  //////////////////////// Set PID parameters ///////////////
  robot -> _pid.Kp = 53.5f;   // Initial value 55.5f
  robot -> _pid.Ki = 0.0f;
  robot -> _pid.Kd = 0.03f;    ///       0.04

 
  robot -> _GAP_TIMEOUT = 2000;
  robot -> _baseSpeed = 100;    //// 80
  robot -> _isFirstCall = false;
  // arrowCount = 1;

  // previousDirection = 1;
  // Set the last time
  robot -> _completeLineLost = false;
  robot -> _lastTime = millis();

}



//////////////////////// Sub tasks implementation ///////////////
bool RampCrossing::_makeTheBridge(){
      float baseSpeed = robot -> _baseSpeed;

      delay(50);

      robot -> _steppers.runForward(baseSpeed, 750);
      delay(100);
      robot -> _steppers.runForward(-baseSpeed, 700);
      delay(100);

      for (int i = 0; i < 1; i++) robot-> turnBack();

      // delay(5);
      robot -> _steppers.runForward(baseSpeed, 70);

      delay(100);
      robot -> _lastTime = millis();
      return true;

}

//////////////////////////////////////////////////////////////////////////////////////////
bool RampCrossing::_crossTheRamp(){
    // makeTheBridge();
  float baseSpeed = robot -> _baseSpeed; 

  // Run forward until the next perpendicular line is found
  robot -> _steppers.runForward(baseSpeed, 150);

  robot -> _irsensors.read();
  robot -> detectPerpendicularLine = robot -> _tracker.isPerpendicularLine(robot -> _irsensors.values);  // Look for the perpendicula line

  while(!(robot->detectPerpendicularLine)){
    robot -> _steppers.setSpeed(baseSpeed, baseSpeed);
    robot -> _steppers.runMotors();
    delay(5);

    robot -> _irsensors.read();
  robot -> detectPerpendicularLine = robot -> _tracker.isPerpendicularLine(robot -> _irsensors.values);
  }

  robot ->alignWithPerpendicularLine();

  delay(100);
  robot -> _steppers.runForward(-baseSpeed, 900);
  delay(100);
  // turnBack();
  robot -> turn90DegreesLeft();

  delay(1000);

  // robot -> _steppers.runSteps(-1500, -1500);

  // Go until the line is found //////////////////////////////////////
  robot -> _steppers.runForward(-baseSpeed, 600);

  robot ->_irsensors.read();
  float pos = robot -> _tracker.computePosition(robot -> _irsensors.values);

  robot -> _steppers.setSpeed(-baseSpeed, -baseSpeed);

  while(pos == 999.0f){
    robot -> _steppers.runMotors();

    robot ->_irsensors.read();
    pos = robot -> _tracker.computePosition(robot -> _irsensors.values);

    delay(5);
  } ///////////////////////////////////////////////

  // robot ->alignWithPerpendicularLine();
  // robot ->turnBack();
  // robot -> _steppers.runForward(-baseSpeed, 1800);
  robot -> _steppers.stop();
  delay(100);

  return true;
};

///////////////////////////////////// Ramp crossing ////////////////////////////////////////////////
void RampCrossing::rampCrossing(){
  if(subTasksNavigation[0]){   // Sub task 1 //////////////////

          // Follow the dash line until the perpendicular line is reached
          robot -> _endOfTheTask = robot -> dashedLineFollow(-1);

          // If the perpendicular line is reached, go to the next sub task
          if (robot -> _endOfTheTask){
            subTasksNavigation[0] = false;
            subTasksNavigation[1] = true;

            robot -> _endOfTheTask = false;   // Reset the variable for the next sub task
          }

  } else if (subTasksNavigation[1]){  // Sub task 2 ///////////////
        // Align the robot with the perpendicular line, Make the bridge, 
        //come back to perpendicular line and turn
        robot -> _endOfTheTask = _makeTheBridge();

        // If the perpendicular line is reached, go to the next sub task
        if (robot -> _endOfTheTask){
          subTasksNavigation[1] = false;
          subTasksNavigation[2] = true;
          robot -> _endOfTheTask = false;   // Reset the variable for the next sub task

        }

  } else if (subTasksNavigation[2]){ // Sub tsk 3 ////////////////
        // Follow the dash line and reach the begining of the task
         robot -> _endOfTheTask = robot -> dashedLineFollow(1);

        // If the perpendicular line is reached, go to the next sub task
        if (robot -> _endOfTheTask){
          subTasksNavigation[2] = false;
          subTasksNavigation[3] = true;

          robot -> _endOfTheTask = false;   // Reset the variable for the next sub task

        }
  } else if (subTasksNavigation[3]){ // Sub task 4 ////////////////
        // Cross the ramp and reach the perpendicular line
        robot -> _endOfTheTask = _crossTheRamp();
        
        // If the perpendicular line is reached, go to the next sub task
        if (robot -> _endOfTheTask){
          subTasksNavigation[3] = false;
          robot -> _steppers.stop();
          // rampSubTasksNavigation[3] = true;

          robot -> _endOfTheTask = false;   // Reset the variable for the next sub task

        }

  }

  // Another function is needed to align with the perpendicular line
  // Add the above functionality.....    
}