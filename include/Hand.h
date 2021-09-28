#ifndef HAND_H
#define HAND_H

#include <Motor.h>

class Hand {
   private:
    Motor _motor;
    Debug* _debug;
    char _msg[100];
    int _calib_position[4];
    int* _presentValue;

   public:
    Hand(Motor& motor, Debug* debug);
    void init();
    bool calibratePosition();
    bool calibratePositionBulk();
    bool movePositionRelative(int val[]);
    int* getCalibrationValue();
};

#endif