#ifndef ARROW_FOLLOWING_H
#define ARROW_FOLLOWING_H

class Robot;

class ArrowFollowing{
  public:
      ArrowFollowing(Robot *robotPtr);

      void initArrowFollowing();

      bool arrowFollowing();

      // Arrtibutes
      int arrowCount;
      int previousDirection;

   private:
      Robot* robot;

      

      // Methods ///////////////
      bool _followTheArrow(); 
      //// Go back to arrow is available in robot class, No need to implement again
};

#endif