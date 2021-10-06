#ifndef HAND_H
#define HAND_H

#include <Motor.h>

class Hand {
   private:
    Motor _motor;
    Debug* _debug;
    char _msg[100];
    int _calib_position[5];
    int _limit_min[5];
    int _limit_max[5];
    int* _present_value;

   public:
    Hand(Motor& motor, Debug* debug);
    void init();
    
    bool calibratePosition();
    bool calibratePositionBulk();
    bool movePosition(int val[]);
    bool movePositionRelative(int val[]);
    bool movePositionRelativePrecision(int val[]);

    void updatePresentValue();
    int* getCalibrationValue();
    int* getPresentValue();
};

#endif