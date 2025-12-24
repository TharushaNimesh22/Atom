#ifndef SIDE_ARM_H
#define SIDE_ARM_H

#include <Arduino.h>

class SideArm{
  public:
    // Attrobutes
    const long numStepsPerRev = 4096;  // 5.625° per step; Gear reduction ratio: 1:64
    // SideArm();

    // Public Methods
    void init();
    // void pickBall();
    // void printPosition();
    void moveArm(int direction, int steps);
    void rotateFull(int direction);


  private:

    //Private attributes
    
    const long _upperPosition = numStepsPerRev / 3;  // 120 degrees
    // long _pos;
   
    const int _pins[4] = {39, 40, 41, 42};
    const int _stepDelay = 3;
    const int _stepSequence[8][4] = {
            {1, 0, 0, 0},
            {1, 1, 0, 0},
            {0, 1, 0, 0},
            {0, 1, 1, 0},
            {0, 0, 1, 0},
            {0, 0, 1, 1},
            {0, 0, 0, 1},
            {1, 0, 0, 1}
          };


    //Private methods
    // void _moveArm(int direction, int steps);
    void _stepMotor(int direction); 
};
#endif