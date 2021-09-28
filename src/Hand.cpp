#include <Hand.h>
#include <Motor.h>

Hand::Hand(Motor& motor, Debug* debug) : _motor(motor), _debug(debug), _msg("") {
}

// Initialize class
void Hand::init() {
    _debug->println("Hand::init");
    _motor.setOperatingMode(OP_EXTENDED_POSITION);

    for (unsigned int i = 0; i < sizeof(Hand::_calib_position) / sizeof(Hand::_calib_position[0]); i++) {
        Hand::_calib_position[i] = 0;
    }
    if (Hand::calibratePositionBulk()) {
    }
}

// Home motor to base position (one-by-one)
bool Hand::calibratePosition() {
    static double current_limit[_motor.DXL_ID_CNT] = {100, 100, 70, 70};  // limit to stop calibration movement
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
bool Hand::calibratePositionBulk() {
    const double current_limit[_motor.DXL_ID_CNT] = {70, 70, 70, 70};  // limit to stop calibration movement
    const float delta = 10;
    int value[_motor.DXL_ID_CNT] = {0};
    int positions[_motor.DXL_ID_CNT] = {0};
    int currents[_motor.DXL_ID_CNT] = {0};

    bool motor_flag[_motor.DXL_ID_CNT] = {0};
    bool calibration_flag = false;

    while (!calibration_flag) {
        Hand::_presentValue = _motor.getPresentValues();
        for (unsigned int i = 0; i < _motor.DXL_ID_CNT; i++) {
            currents[i] = *(Hand::_presentValue + i);
            positions[i] = *(Hand::_presentValue + i + _motor.DXL_ID_CNT);
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

    // set calibration value
    Hand::_presentValue = _motor.getPresentValues();
    for (int i = 0; i < _motor.DXL_ID_CNT; i++) {
        _calib_position[i] = *(Hand::_presentValue + i + _motor.DXL_ID_CNT);
    }
    return true;
}

int* Hand::getCalibrationValue() {
    return Hand::_calib_position;
}

bool Hand::movePositionRelative(int val[]) {
    const double current_limit[_motor.DXL_ID_CNT] = {70, 70, 70, 70};  // limit to stop movement
    const float delta = 10;

    int goal_position[_motor.DXL_ID_CNT] = {0};
    int value[_motor.DXL_ID_CNT] = {0};
    int positions[_motor.DXL_ID_CNT] = {0};
    int currents[_motor.DXL_ID_CNT] = {0};

    bool motor_flag[_motor.DXL_ID_CNT] = {0};
    bool movement_flag = false;
    bool force_stop_flag = false;

    for (int i = 0; i < _motor.DXL_ID_CNT; i++) {
        goal_position[i] = *(_presentValue + _motor.DXL_ID_CNT + i) + val[i];
    }

    while (!movement_flag) {
        Hand::_presentValue = _motor.getPresentValues();
        for (unsigned int i = 0; i < _motor.DXL_ID_CNT; i++) {
            currents[i] = *(Hand::_presentValue + i);
            positions[i] = *(Hand::_presentValue + i + _motor.DXL_ID_CNT);
        }

        for (unsigned int i = 0; i < _motor.DXL_ID_CNT; i++) {
            value[i] = positions[i];
            if (currents[i] < current_limit[i]) {
                if (positions[i] < goal_position[i]) {
                    value[i] = (i % 2 == 0) ? value[i] - delta : value[i] + delta;
                } else {
                    motor_flag[i] = true;
                }
            } else {
                motor_flag[i] = true;
                force_stop_flag = true;
            }
        }

        if (!force_stop_flag)  // move only if the current motor is below the current limit
            _motor.setGoalPositions(value);

        int j = 0;
        for (int i = 0; i < _motor.DXL_ID_CNT; i++) {
            if (motor_flag[i])
                j++;
        }
        if (j >= _motor.DXL_ID_CNT || force_stop_flag == true)
            movement_flag = true;
    }

    return !force_stop_flag;
}