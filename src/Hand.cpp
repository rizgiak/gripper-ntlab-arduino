#include <Hand.h>
#include <Motor.h>

Hand::Hand(Motor& motor, Debug* debug) : _motor(motor), _debug(debug), _msg("") {
}

// Initialize class
void Hand::init() {
    _debug->println("Hand::init");
    _motor.setOperatingMode(OP_EXTENDED_POSITION);
    if (Hand::CalibratePositionBulk()) {
    }
}

// Home motor to base position (one-by-one)
bool Hand::CalibratePosition() {
    static double current_limit[_motor.DXL_ID_CNT] = {100, 100, 70, 70}; // limit to stop calibration movement
    static float delta = 20;
    for (int i = 0; i < _motor.DXL_ID_CNT; i++) {
        while (_motor.getPresentCurrent(i) < current_limit[i]) {
            if (i % 2 == 0) {
                _motor.setGoalPosition(i, _motor.getPresentPosition(i) - delta);
            } else {
                _motor.setGoalPosition(i, _motor.getPresentPosition(i) + delta);
            }
            delay(10);
        }
    }
    return true;
}

// Home motor to base position (parallel)
bool Hand::CalibratePositionBulk() {
    const double current_limit[_motor.DXL_ID_CNT] = {70, 70, 70, 70};  // limit to stop calibration movement
    const float delta = 10;
    int value[_motor.DXL_ID_CNT] = {0};
    int* results;
    int positions[_motor.DXL_ID_CNT] = {0};
    int currents[_motor.DXL_ID_CNT] = {0};

    bool motor_flag[_motor.DXL_ID_CNT] = {0};
    bool calibration_flag = false;

    while (!calibration_flag) {
        results = _motor.getPresentValues();
        for (unsigned int i = 0; i < _motor.DXL_ID_CNT; i++) {
            currents[i] = *(results + i);
            positions[i] = *(results + i + _motor.DXL_ID_CNT);
        }

        for (unsigned int i = 0; i < _motor.DXL_ID_CNT; i++) {
            value[i] = positions[i];
            if (currents[i] < current_limit[i]) {
                value[i] = (i % 2 == 0) ? value[i] - delta : value[i] + delta;
            } else {
                motor_flag[i] = true;
            }
        }

        _motor.setGoalPositions(value);
        int j = 0;
        for (int i = 0; i < _motor.DXL_ID_CNT; i++) {
            if (motor_flag[i])
                j++;
        }
        if (j >= _motor.DXL_ID_CNT)
            calibration_flag = true;
    }
    return true;
}