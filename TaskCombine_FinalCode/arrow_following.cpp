#include "arrow_following.h"
#include "robot.h"

ArrowFollowing::ArrowFollowing(Robot* robotPtr) {
  this -> robot = robotPtr; // save the same robot pointe)
}

void ArrowFollowing::initArrowFollowing(){
      arrowCount = 1;
      previousDirection = 1;

      robot -> detectPerpendicularLine = false;
        //////////////////////// Set PID parameters ///////////////
      robot -> _pid.Kp = 55.5f;   // Initial value 55.5f   53.5
      robot -> _pid.Ki = 0.0f;
      robot -> _pid.Kd = 0.03f;    ///       0.03           0.05


      robot -> _GAP_TIMEOUT = 5500;
      robot -> _baseSpeed = 80;    //// 80
      robot -> _isFirstCall = false;
      robot -> _lineVisible = true;
     
      // Set the last time
      robot -> _lastTime = millis();

}


//////////////////////  Main function(public) /////////////
bool ArrowFollowing::arrowFollowing(){
  if (!(robot -> _completeLineLost)){
      return _followTheArrow();

    }else{
       return robot -> _goBackToLine(previousDirection);
    }
}

///////////////////////////////////////
// bool ArrowFollowing::_followTheArrow(){    // If the arrow is available, follow the arrow
//     // Run PID loop at 10ms
//   if (millis() - (robot -> _lastTime) >= (robot -> _LOOP_DT) * 1000) {
//     robot -> _lastTime += (robot -> _LOOP_DT) * 1000;

//     // Read IR sensor values
//     robot -> _irsensors.read();


//     robot -> detectPerpendicularLine = robot -> _tracker.isPerpendicularLine(robot -> _irsensors.values);
//     if (robot -> detectPerpendicularLine){
//       // If perpendicular line is detetcted, stop the line follow
//         robot -> stopTheRobot(false);

//         robot -> detectPerpendicularLine = false;           ////////////  reset in the other places as well ///////////
//         return true;
//     }
//     // Compute the line position
//     float pos = robot -> _tracker.computePosition(robot -> _irsensors.values);
    
//     // If line is found
//     if (pos != 999.0f) {

//         ////////////////// Check if the line is detected for the first time /////////////// 
//         if (!(robot -> _lineVisible)){
//           /// If the first time, need to update the previous direction, and may want to take a sharp turn

//           if(arrowCount == 11){   // Last arrow is reached
//              // If perpendicular line is detetcted, stop the line follow
//                 int maxCount = 0;
                
//                 ///  align with the line
//                 while (maxCount < 30){
//                       robot -> _irsensors.read();

//                       if (robot -> _irsensors.values[0] == 1 && robot -> _irsensors.values[4] == 1){
//                         robot -> stopTheRobot(false);
//                         break;
//                       } else if (robot -> _irsensors.values[0] == 0 && robot -> _irsensors.values[4] == 1){
//                           robot-> _steppers.setSpeed(70, 10);
//                           robot-> _steppers.runMotors();
//                       } else if (robot -> _irsensors.values[0] == 1 && robot -> _irsensors.values[4] == 0){
//                           robot-> _steppers.setSpeed(10, 70);
//                           robot-> _steppers.runMotors();
//                       } else{
//                           robot-> _steppers.setSpeed(70, 70);
//                           robot-> _steppers.runMotors();
//                       }

//                       delay(100);
//                 }
                
//                 robot -> alignWithPerpendicularLine();
//                 robot -> stopTheRobot(false);
//                 robot -> detectPerpendicularLine = false;           ////////////  reset in the other places as well ///////////
                
//                 return true;

//           }

//           /// Update arrow count
//           arrowCount++;

          

//           if (arrowCount == 3) {
//                 // turn left (small turn: 90 degrees)
//                 previousDirection = 1;
//                 // if(arrowCount == 5) robot -> _steppers.runForward(robot -> _baseSpeed, 150);
//                 robot -> sharpTurn(previousDirection);  // Sharp turn to left

//                 robot -> _lastPos = pos;
//                 return false;   // go to the start of the funtion and recalculate the position

//           } else if (arrowCount == 5){

//                 // turn right (small turn: 90 degrees)
//                 previousDirection = -1;
//                 robot -> _steppers.runForward(robot -> _baseSpeed, 150);
//                 robot -> sharpTurn(previousDirection);       // Sharp turn to right   

//                 robot -> _lastPos = pos;

               
                
//                 return false;
//           } 

//           // if (arrowCount == 4) {
//           //         previousDirection = -1;

//           //         robot -> _steppers.turn(50, 100, 80);         /////////////////// temp
//           // } else if (arrowCount == 5){
//           //         previousDirection = 1;
//           // }
          
//         }


//         // Line is visible and not the first time (not the first time after the TIME OUT period)
//         robot -> _lineVisible = true;
//         robot -> _lastSeenTime = millis();
//         robot -> _lastPos = pos;
//     }
//     // If line is not found
//     else {
//         robot -> _lineVisible = false;
//     }

//     // Handle dashed line
//     if (!(robot -> _lineVisible) && (millis() - robot -> _lastSeenTime < robot -> _GAP_TIMEOUT)) {
//         // Continue with the previous ir tracks
//         pos = robot -> _lastPos;
//     }

//     // Handle complete line lost
//     if (!(robot -> _lineVisible) && (millis() - (robot -> _lastSeenTime) >= (robot -> _GAP_TIMEOUT))) {
//         // Line lost fallback
//         robot -> stopTheRobot(true);
//         delay(1000);
//         return false;
//     }

//     // Update PID Controller
//     float setpoint = 0.0f;
//     float error = setpoint - pos;

//     robot -> _steering = robot -> _pid.update(setpoint, pos);
    
//     // Update the motor X, Y speeds
//     float right = (robot -> _baseSpeed) + (robot -> _steering);
//     float left = (robot -> _baseSpeed) - (robot -> _steering);

//     // Clamp motor commands
//     if (left > 255) left = 255;
//     if (left < -255) left = -255;
//     if (right > 255) right = 255;
//     if (right < -255) right = -255;
    
//     //Run the motors
//     robot -> _steppers.setSpeed(left, right);
//     robot -> _steppers.runMotors();
//   }

//   return false;
// }

//////////////////////////////////////
bool ArrowFollowing::_followTheArrow(){    // If the arrow is available, follow the arrow
    // Run PID loop at 10ms
  if (millis() - (robot -> _lastTime) >= (robot -> _LOOP_DT) * 1000) {
    robot -> _lastTime += (robot -> _LOOP_DT) * 1000;

    // Read IR sensor values
    robot -> _irsensors.read();


    // robot -> detectPerpendicularLine = robot -> _tracker.isPerpendicularLine(robot -> _irsensors.values);
    // if (robot -> detectPerpendicularLine){
    //   // If perpendicular line is detetcted, stop the line follow
    //     robot -> stopTheRobot(false);

    //     robot -> detectPerpendicularLine = false;           ////////////  reset in the other places as well ///////////
    //     return true;
    // }
    // Compute the line position
    float pos = robot -> _tracker.computePosition(robot -> _irsensors.values);
    
    // If line is found
    if (pos != 999.0f) {

        ////////////////// Check if the line is detected for the first time /////////////// 
        if (!robot -> _lineVisible){
          /// If the first time, need to update the previous direction, and may want to take a sharp turn

          if(arrowCount == 11){   // Last arrow is reached
             // If perpendicular line is detetcted, stop the line follow
                robot -> stopTheRobot(false);
                robot -> detectPerpendicularLine = false;           ////////////  reset in the other places as well ///////////
                
                return true;

          }

          /// Update arrow count
          arrowCount++;

          if ((robot -> _irsensors.values[0] && robot -> _irsensors.values[1] && robot -> _irsensors.values[2]) || 
              (robot -> _irsensors.values[0] && robot -> _irsensors.values[1] ) || 
              (robot -> _irsensors.values[0])) {
                // turn left (small turn: 90 degrees)
                previousDirection = -1;
                robot -> sharpTurn(previousDirection);  // Sharp turn to left

                robot -> _lastPos = pos;
                return false;   // go to the start of the funtion and recalculate the position

          } else if ((robot -> _irsensors.values[4] && robot -> _irsensors.values[3] && robot -> _irsensors.values[2]) || 
                    (robot -> _irsensors.values[4] && robot -> _irsensors.values[3] ) || 
                    (robot -> _irsensors.values[4])){

                // turn right (small turn: 90 degrees)
                previousDirection = 1;
                robot -> sharpTurn(previousDirection);       // Sharp turn to right   

                robot -> _lastPos = pos;
                return false;
          } 
        }


        // Line is visible and not the first time (not the first time after the TIME OUT period)
        robot -> _lineVisible = true;
        robot -> _lastSeenTime = millis();
        robot -> _lastPos = pos;
    }
    // If line is not found
    else {
        robot -> _lineVisible = false;
    }

    // Handle dashed line
    if (!(robot -> _lineVisible) && (millis() - robot -> _lastSeenTime < robot -> _GAP_TIMEOUT)) {
        // Continue with the previous ir tracks
        pos = robot -> _lastPos;
    }

    // Handle complete line lost
    if (!(robot -> _lineVisible) && (millis() - (robot -> _lastSeenTime) >= (robot -> _GAP_TIMEOUT))) {
        // Line lost fallback
        robot -> stopTheRobot(true);
        delay(1000);
        return false;
    }

    // Update PID Controller
    float setpoint = 0.0f;
    float error = setpoint - pos;

    robot -> _steering = robot -> _pid.update(setpoint, pos);
    
    // Update the motor X, Y speeds
    float right = (robot -> _baseSpeed) + (robot -> _steering);
    float left = (robot -> _baseSpeed) - (robot -> _steering);

    // Clamp motor commands
    if (left > 255) left = 255;
    if (left < -255) left = -255;
    if (right > 255) right = 255;
    if (right < -255) right = -255;
    
    //Run the motors
    robot -> _steppers.setSpeed(left, right);
    robot -> _steppers.runMotors();
  }

  return false;
}

