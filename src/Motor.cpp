#include <Dynamixel2Arduino.h>
#include <Motor.h>

using namespace ControlTableItem;

Motor::Motor(Dynamixel2Arduino& dxl, unsigned long baudrate, float version, Debug* debug) : _dxl(dxl), _baudrate(baudrate), _version(version), _debug(debug) {
}

void Motor::init() {
    _debug->println("Motor::init");
    _dxl.begin(_baudrate);
    _dxl.setPortProtocolVersion(_version);
}

bool Motor::setOperatingMode(int id, OperatingMode mode) {
    bool ret = false;
    if (_dxl.ping(id)) {
        ret = _dxl.torqueOff(id);
        if (ret) ret = _dxl.setOperatingMode(id, mode);
        if (ret) ret = _dxl.torqueOn(id);
    }
    return ret;
}

bool Motor::setPID(int id, int p, int i, int d) {
    bool ret = false;
    ret = _dxl.writeControlTableItem(POSITION_P_GAIN, id, p);
    if (ret) ret = _dxl.writeControlTableItem(POSITION_I_GAIN, id, i);
    if (ret) ret = _dxl.writeControlTableItem(POSITION_D_GAIN, id, d);
    return ret;
}

bool Motor::setVelocity(int id, int vel) {
    return _dxl.writeControlTableItem(PROFILE_VELOCITY, id, vel);
}

float Motor::getPresentPosition(int id) {
    return _dxl.getPresentPosition(id);
}

float Motor::getPresentCurrent(int id) {
    return abs(_dxl.getPresentCurrent(id));
}

bool Motor::setGoalPosition(int id, float val) {
    return _dxl.setGoalPosition(id, val);
}

bool Motor::setGoalPositions(int id[], int val[]) {
    ParamForBulkWriteInst_t bulk_write_param;
    for (unsigned int i = 2; i < 3; i++) {
        static int32_t position = val[i];
        bulk_write_param.xel[i].id = id[i];
        bulk_write_param.xel[i].addr = 116;  //Goal Position on X serise
        bulk_write_param.xel[i].length = 4;
        memcpy(bulk_write_param.xel[i].data, &position, sizeof(position));
    }
    bulk_write_param.id_count = 1;
    return _dxl.bulkWrite(bulk_write_param);
}

int* Motor::getPresentCurrents(int id[]) {
    return getValues(id, 126);
}

int* Motor::getPresentPositions(int id[]) {
    return getValues(id, 132);
}

int* Motor::getValues(int id[], uint16_t address) {
    ParamForBulkReadInst_t bulk_read_param;
    RecvInfoFromStatusInst_t read_result;
    static int results[Motors::SIZE];
    int result;

    for (unsigned int i = 0; i < Motors::SIZE; i++) {
        bulk_read_param.xel[i].id = id[i];
        bulk_read_param.xel[i].addr = address;  //Present Current on X serise
        bulk_read_param.xel[i].length = 4;
    }
    bulk_read_param.id_count = Motors::SIZE;
    _dxl.bulkRead(bulk_read_param, read_result);

    for (unsigned int i = 0; i < Motors::SIZE; i++) {
        memcpy(&result, read_result.xel[i].data, read_result.xel[i].length);
        results[i] = result;
    }
    return results;
}