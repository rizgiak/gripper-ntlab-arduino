#ifndef MOTOR_H
#define MOTOR_H

#include <Debug.h>
#include <Dynamixel2Arduino.h>
#include <Vector.h>

enum Motors {
    LEFT_HAND,
    RIGHT_HAND,
    LEFT_FINGER,
    RIGHT_FINGER,
    SIZE,
    TIP
};

class Motor {
   private:
    Dynamixel2Arduino _dxl;
    unsigned long _baudrate;
    float _version;
    Debug* _debug;

    int* getValues(int id[], uint16_t address);

   public:
    Motor(Dynamixel2Arduino& dxl, unsigned long baudrate, float version, Debug* debug);
    void init();
    bool Move(int ID, int position);
    bool setOperatingMode(int id, OperatingMode mode);
    bool setPID(int id, int p, int i, int d);
    bool setVelocity(int id, int vel);
    float getPresentPosition(int id);
    int* getPresentPositions(int id[]);
    float getPresentCurrent(int id);
    int* getPresentCurrents(int id[]);
    bool setGoalPosition(int id, float val);
    bool setGoalPositions(int id[], int val[]);
};

#endif