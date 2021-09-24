#ifndef MOTOR_H
#define MOTOR_H

#include <Debug.h>
#include <Dynamixel2Arduino.h>

class Motor {
   private:
    Dynamixel2Arduino _dxl;
    Debug* _debug;
    char _msg[100];

   public:
    // Number of IDs
    static const uint8_t DXL_ID_CNT = 4;
    // List of IDs
    const uint8_t DXL_ID_LIST[Motor::DXL_ID_CNT] = {0, 1, 2, 3};

    Motor(Debug* debug);
    void init();
    bool setOperatingMode(OperatingMode mode);
    bool setPID(int id, int p, int i, int d);
    bool setVelocity(int id, int vel);
    float getPresentPosition(int id);
    int* getPresentPositions(int id[]);
    float getPresentCurrent(int id);
    int* getPresentCurrents(int id[]);
    bool setGoalPosition(int id, float val);
    bool setGoalPositions(int val[]);
    int* getPresentValues();
};

#endif