#ifndef RAMP_CROSSING_H
#define RAMP_CROSSING_H

class Robot;

class RampCrossing{
    public:
      RampCrossing(Robot* robotPtr);

      //// Attributes ///////////////
      bool subTasksNavigation[4];    // Length 4

      /////////// Methods ///////////
      void initRampCrossing();    // Done

      void rampCrossing();

    private:
      Robot* robot;

      bool _makeTheBridge();   // Function to make the bridge
      bool _crossTheRamp();    

      // Another function is needed to align with the perpendicular lu
      

};

#endif