#include <Hand.h>
#include <Motor.h>

Hand::Hand(Motor& motor, Debug* debug) : _motor(motor), _debug(debug), _msg("") {
}

// Initialize class
void Hand::init() {
    _debug->println("Hand::init");
    for (int i = 0; i < Motors::SIZE; i++) {
        // set to multi-turn mode
        _motor.setOperatingMode(i, OP_EXTENDED_POSITION);
    }
    if (Hand::CalibratePositionBulk()) {
    }
}

// Home motor to base position (one-by-one)
bool Hand::CalibratePosition() {
    static double current_limit[Motors::SIZE] = {100,
                                                 100,
                                                 70,
                                                 70};
    static float delta = 20;
    for (int i = 0; i < Motors::SIZE; i++) {
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
    static double current_limit[Motors::SIZE] = {100,
                                                 100,
                                                 70,
                                                 70};
    static float delta = 20;
    int motors[Motors::SIZE] = {0, 1, 2, 3};
    int value[Motors::SIZE] = {0, 0, 0, 0};
    int* results;
    int positions[Motors::SIZE] = {0, 0, 0, 0};
    int currents[Motors::SIZE] = {0, 0, 0, 0};

    bool calibration_flag = true;
    bool res = _motor.setGoalPositions(motors, value, Motors::SIZE);
    // while (calibration_flag) {
    //     calibration_flag = false;
    //     results = _motor.getPresentCurrents(motors);
    //     for(unsigned int i = 0; i < Motors::SIZE; i++){
    //         currents[i] = *(results +i);
    //     }
    //     results = _motor.getPresentPositions(motors);
    //     for(unsigned int i = 0; i < Motors::SIZE; i++){
    //         positions[i] = *(results +i);
    //     }
    //     sprintf(_msg, "pos0 %d %d pos1 %d %d", positions[0], currents[0], positions[1], currents[1]);
    //     _debug->println(_msg);
    //     for (unsigned int i = 0; i < Motors::SIZE; i++) {
    //         value[i] = positions[i];
    //         if (currents[i] < current_limit[i]) {
    //             calibration_flag = true;
    //             value[i] = (i % 2 == 0) ? value[i] - delta : value[i] + delta;
    //         }
    //     }
    //     sprintf(_msg, "m0 %d %d m1 %d %d", motors[0], value[0], motors[1], value[1]);
    //     _debug->println(_msg);
        
    //     bool res = _motor.setGoalPositions(motors, value, Motors::SIZE);
    //     sprintf(_msg, "z %d", res);
    //     _debug->println(_msg);
        
    //     delay(1);
    // }
    return true;
}