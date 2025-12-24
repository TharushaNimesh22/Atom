#ifndef FRONT_ARM_H
#define FRONT_ARM_H

#include <Servo.h>
#include <Arduino.h>


#define FRONT_ARM_INITIAL_POSITION 70


class FrontArm {
  public:
        // const int initialPos = 70;
      // Consrtuctor
      // Arm();  // Implement later, for now use the defalut constructor and destructor

      void init(int pin);

      // Catch and release functions
        void armForward(int pos);
        void armBackward(int pos);
        void initialPosition();

      void catchBox();
      void releaseBox();


      //Optional
      void printPosition();

  private:
      Servo _myServo; 
      int _pos;  //Save the current position of the arm
      // const int _initialPos = 70;

};

#endif