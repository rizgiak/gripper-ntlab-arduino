#ifndef HAND_H
#define HAND_H

#include <Motor.h>

class Hand {
    private:
        Motor _motor;
        Debug* _debug;
        char _msg[100];

    public:
        Hand(Motor &motor, Debug* debug);
        void init();
        bool CalibratePosition();
        bool CalibratePositionBulk();

};

#endif