#ifndef WALL_FOLLOWING_H
#define WALL_FOLLOWING_H

class Robot;

class WALLFollowing{
    public:
      WALLFollowing(Robot *robotPtr);

      void initCicrularWallFollowing();
      bool runCircularWallFollowing();

      void initStraightWallFollowing();
      bool runStraightWallFollowing();


      void comeBack();

    private:
      Robot *robot;

      ////////////////////////////////////////
      /// parameters for circular wall following
      float _maxWallDist;
      float _minWallDist;
      int sensor;
      const int _stepsFor90DegreeTurn = 260;
      float distance;
      int tm_count;


      void circularWallFollowing();
      



};


#endif