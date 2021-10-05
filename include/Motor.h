#ifndef MOTOR_H
#define MOTOR_H

#include <Debug.h>
#include <Dynamixel2Arduino.h>

class Motor {
   private:
    Dynamixel2Arduino _dxl;
    Debug* _debug;
    char _msg[100];
    int _servo_position;

   public:
    // Number of IDs
    static const uint8_t DXL_ID_CNT = 4;
    // List of IDs
    const uint8_t DXL_ID_LIST[Motor::DXL_ID_CNT] = {0, 1, 2, 3};

    Motor(Debug* debug);
    void init();
    bool initServo();
    bool move(int val);
    bool setOperatingMode(OperatingMode mode);
    bool setTorque(bool torque);
    bool setPID(int id, int p, int i, int d);
    bool setVelocity(int id, int vel);
    bool setGoalPosition(int id, float val);
    bool setGoalPositions(int val[]);

    float getPresentCurrent(int id);
    float getPresentPosition(int id);
    int* getPresentValues();
    int* getPresentCurrents(int id[]);
    int* getPresentPositions(int id[]);
};

#endif