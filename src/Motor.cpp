#include <Dynamixel2Arduino.h>
#include <Motor.h>

const uint8_t BROADCAST_ID = 254;
const uint8_t DXL_ID_CNT = 2;
const uint8_t DXL_ID_LIST[DXL_ID_CNT] = {1, 2};

using namespace ControlTableItem;

typedef struct sw_data {
    int32_t goal_velocity;
} __attribute__((packed)) sw_data_t;

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

//https://github.com/ROBOTIS-GIT/Dynamixel2Arduino/blob/master/examples/advanced/sync_read_write_raw/sync_read_write_raw.ino
bool Motor::setGoalPositions(int id[], int val[], unsigned int size) {
    sw_data_t sw_data[DXL_ID_CNT];
    DYNAMIXEL::InfoSyncWriteInst_t sw_infos;
    DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw[DXL_ID_CNT];

    for (int i = 0; i < DXL_ID_CNT; i++) {
        _dxl.torqueOff(DXL_ID_LIST[i]);
        _dxl.setOperatingMode(DXL_ID_LIST[i], OP_VELOCITY);
    }
    _dxl.torqueOn(BROADCAST_ID);

    // Fill the members of structure to syncWrite using internal packet buffer
    sw_infos.packet.p_buf = nullptr;
    sw_infos.packet.is_completed = false;
    sw_infos.addr = 104;
    sw_infos.addr_length = 4;
    sw_infos.p_xels = info_xels_sw;
    sw_infos.xel_count = 0;

    sw_data[0].goal_velocity = 0;
    sw_data[1].goal_velocity = 100;
    for (int i = 0; i < DXL_ID_CNT; i++) {
        info_xels_sw[i].id = DXL_ID_LIST[i];
        info_xels_sw[i].p_data = (uint8_t*)&sw_data[i].goal_velocity;
        sw_infos.xel_count++;
    }
    sw_infos.is_info_changed = true;

    while (1) {
        for (int i = 0; i < DXL_ID_CNT; i++) {
            sw_data[i].goal_velocity = 5 + sw_data[i].goal_velocity;
            if (sw_data[i].goal_velocity >= 50) {
                sw_data[i].goal_velocity = 0;
            }
        }
        sw_infos.is_info_changed = true;
        if (_dxl.syncWrite(&sw_infos) == true) {
            _debug->println("[SyncWrite] Success");
        } else {
            _debug->print("[SyncWrite] Fail, Lib error code: ");
        }

        delay(250);
    }
    return true;
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