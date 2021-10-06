#include <Hand.h>
#include <Motor.h>

#define AUTO_CALIBRATION true

Hand::Hand(Motor& motor, Debug* debug) : _motor(motor), _debug(debug), _msg("") {
}

// Initialize class
void Hand::init() {
    _debug->println("Hand::init");
    _motor.setOperatingMode(OP_EXTENDED_POSITION);

    for (unsigned int i = 0; i < _motor.DOF; i++) {
        _calib_position[i] = 0;
        _motor.setVelocity(i, 100);
    }

#if AUTO_CALIBRATION
    _motor.setTorque(true);
    Hand::calibratePositionBulk();
#else
    _motor.setTorque(false);
#endif
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
    const int limit_offset[_motor.DOF] = {3600, 3600, 1200, 1200, 2000}; // limit offset each motors
    const float delta = 10;
    int value[_motor.DXL_ID_CNT] = {0};
    int positions[_motor.DXL_ID_CNT] = {0};
    int currents[_motor.DXL_ID_CNT] = {0};

    bool motor_flag[_motor.DXL_ID_CNT] = {0};
    bool calibration_flag = false;

    while (!calibration_flag) {
        delay(10);
        _present_value = _motor.getPresentValues();
        for (unsigned int i = 0; i < _motor.DXL_ID_CNT; i++) {
            currents[i] = *(_present_value + i);
            positions[i] = *(_present_value + i + _motor.DOF);
        }

        for (unsigned int i = 0; i < _motor.DXL_ID_CNT; i++) {
            value[i] = positions[i];
            if (currents[i] < current_limit[i] && !motor_flag[i]) {
                value[i] = (i % 2 == 0) ? value[i] - delta : value[i] + delta;
            } else {
                motor_flag[i] = true;
            }
        }
        delay(10);
        _motor.setGoalPositions(value);
        int j = 0;
        for (int i = 0; i < _motor.DXL_ID_CNT; i++) {
            if (motor_flag[i])
                j++;
        }
        if (j >= _motor.DXL_ID_CNT)
            calibration_flag = true;
    }

    _motor.initServo();

    // set calibration value
    _present_value = _motor.getPresentValues();
    _debug->print("calibration result: ");
    for (int i = 0; i < _motor.DOF; i++) {
        _calib_position[i] = *(_present_value + i + _motor.DOF);
        sprintf(_msg, "p%d:%d, ", i, _calib_position[i]);
        _debug->print(_msg);
    }
    _debug->println("");

    //set limit
    for (int i = 0; i < _motor.DOF; i++) {
        if (i % 2 == 0) {
            _limit_min[i] = _calib_position[i];
            _limit_max[i] = _calib_position[i] + limit_offset[i];
        } else {
            _limit_min[i] = _calib_position[i] - limit_offset[i];
            _limit_max[i] = _calib_position[i];
        }
    }
    return true;
}

int* Hand::getCalibrationValue() {
    return _calib_position;
}

int* Hand::getPresentValue() {
    return _present_value;
}

// Get present value from motors
void Hand::updatePresentValue() {
    _present_value = _motor.getPresentValues();
}

// Move position with position limitation
bool Hand::movePosition(int val[]) {
    bool force_stop_flag = false;

    for (unsigned int i = 0; i < _motor.DOF; i++) {
        if (val[i] > _limit_max[i] && val[i] < _limit_min[i]) {
            force_stop_flag = true;
        }
    }

    if (!force_stop_flag) {
        _motor.setGoalPositions(val);
    }
    updatePresentValue();
    return !force_stop_flag;
}

// Move position relative with only position limitation
bool Hand::movePositionRelative(int val[]) {
    const float delta = 10;

    int goal_position[_motor.DOF] = {0};
    int positions[_motor.DOF] = {0};
    bool motor_direction[_motor.DOF] = {0};

    bool motor_flag[_motor.DOF] = {0};
    bool movement_flag = false;
    bool force_stop_flag = false;

    for (int i = 0; i < _motor.DOF; i++) {
        goal_position[i] = *(_present_value + _motor.DOF + i) + val[i];
        if (val[i] > 0)
            motor_direction[i] = true;
    }

    updatePresentValue();
    for (unsigned int i = 0; i < _motor.DOF; i++) {
        positions[i] = *(_present_value + i + _motor.DOF);
    }

    while (!movement_flag) {
        for (unsigned int i = 0; i < _motor.DOF; i++) {
            if (positions[i] <= _limit_max[i] && positions[i] >= _limit_min[i]) {
                if (motor_direction[i]) {  //goal above present value
                    if (positions[i] < goal_position[i]) {
                        positions[i] = positions[i] + delta;
                    } else {
                        motor_flag[i] = true;
                    }
                } else {
                    if (positions[i] > goal_position[i]) {
                        positions[i] = positions[i] - delta;
                    } else {
                        motor_flag[i] = true;
                    }
                }
            } else {
                motor_flag[i] = true;
                force_stop_flag = true;
            }
        }

        if (!force_stop_flag) {  // move only if the current motor is between position limit
            _motor.setGoalPositions(positions);
        }

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

// Move position relative with position and current limit
bool Hand::movePositionRelativePrecision(int val[]) {
    const double current_limit[_motor.DXL_ID_CNT] = {120, 120, 120, 120};  // limit to stop movement
    const float delta = 10;

    int goal_position[_motor.DXL_ID_CNT] = {0};
    int currents[_motor.DXL_ID_CNT] = {0};
    int positions[_motor.DXL_ID_CNT] = {0};
    bool motor_direction[_motor.DXL_ID_CNT] = {0};

    bool motor_flag[_motor.DXL_ID_CNT] = {0};
    bool movement_flag = false;
    bool force_stop_flag = false;

    for (int i = 0; i < _motor.DXL_ID_CNT; i++) {
        goal_position[i] = *(_present_value + _motor.DXL_ID_CNT + i) + val[i];
        if (val[i] > 0)
            motor_direction[i] = true;
    }

    while (!movement_flag) {
        updatePresentValue();
        for (unsigned int i = 0; i < _motor.DXL_ID_CNT; i++) {
            currents[i] = *(_present_value + i);
            positions[i] = *(_present_value + i + _motor.DXL_ID_CNT);
        }

        for (unsigned int i = 0; i < _motor.DXL_ID_CNT; i++) {
            if (currents[i] < current_limit[i]) {
                if (motor_direction[i]) {  //goal above present value
                    if (positions[i] < goal_position[i]) {
                        positions[i] = positions[i] + delta;
                    } else {
                        motor_flag[i] = true;
                    }
                } else {
                    if (positions[i] > goal_position[i]) {
                        positions[i] = positions[i] - delta;
                    } else {
                        motor_flag[i] = true;
                    }
                }
            } else {
                motor_flag[i] = true;
                force_stop_flag = true;
            }
        }

        if (!force_stop_flag) {  // move only if the current motor is below the current limit
            _motor.setGoalPositions(positions);
        }

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