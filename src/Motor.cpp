#include <Motor.h>

#define DXL Serial2
#define DXL_DIR_PIN 2
#define DXL_PROTOCOL_VERSION 2.0
#define DXL_BAUDRATE 57600

const uint16_t user_pkt_buf_cap = 128;
uint8_t user_pkt_buf[user_pkt_buf_cap];

const uint8_t BROADCAST_ID = 254;

const uint16_t SR_START_ADDR = 126;
const uint16_t SR_ADDR_LEN = 10;     //2+4+4
const uint16_t SW_START_ADDR = 116;  //Goal position
const uint16_t SW_ADDR_LEN = 4;

typedef struct sr_data {
    int16_t present_current;
    int32_t present_velocity;
    int32_t present_position;
} __attribute__((packed)) sr_data_t;

typedef struct sw_data {
    int32_t goal_position;
} __attribute__((packed)) sw_data_t;

sr_data_t sr_data[Motor::DXL_ID_CNT];
DYNAMIXEL::InfoSyncReadInst_t sr_infos;
DYNAMIXEL::XELInfoSyncRead_t info_xels_sr[Motor::DXL_ID_CNT];

sw_data_t sw_data[Motor::DXL_ID_CNT];
DYNAMIXEL::InfoSyncWriteInst_t sw_infos;
DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw[Motor::DXL_ID_CNT];

Motor::Motor(Debug* debug) : _dxl(Dynamixel2Arduino(DXL, DXL_DIR_PIN)), _debug(debug) {
}

void Motor::init() {
    _debug->println("Motor::init");
    _dxl.begin(DXL_BAUDRATE);
    _dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

    //Set SyncWrite
    sw_infos.packet.p_buf = nullptr;
    sw_infos.packet.is_completed = false;
    sw_infos.addr = SW_START_ADDR;
    sw_infos.addr_length = SW_ADDR_LEN;
    sw_infos.p_xels = info_xels_sw;
    sw_infos.xel_count = 0;

    for (unsigned int i = 0; i < DXL_ID_CNT; i++) {
        sw_data[i].goal_position = 0;
    }

    for (int i = 0; i < DXL_ID_CNT; i++) {
        info_xels_sw[i].id = DXL_ID_LIST[i];
        info_xels_sw[i].p_data = (uint8_t*)&sw_data[i].goal_position;
        sw_infos.xel_count++;
    }
    sw_infos.is_info_changed = true;

    //Set SyncRead
    sr_infos.packet.p_buf = user_pkt_buf;
    sr_infos.packet.buf_capacity = user_pkt_buf_cap;
    sr_infos.packet.is_completed = false;
    sr_infos.addr = SR_START_ADDR;
    sr_infos.addr_length = SR_ADDR_LEN;
    sr_infos.p_xels = info_xels_sr;
    sr_infos.xel_count = 0;
    for (int i = 0; i < DXL_ID_CNT; i++) {
        info_xels_sr[i].id = DXL_ID_LIST[i];
        info_xels_sr[i].p_recv_buf = (uint8_t*)&sr_data[i];
        sr_infos.xel_count++;
    }
    sr_infos.is_info_changed = true;
}

bool Motor::setOperatingMode(OperatingMode mode) {
    bool ret = true;
    for (int i = 0; i < DXL_ID_CNT; i++) {
        if (ret && _dxl.ping(i)) {
            if (ret) ret = _dxl.torqueOff(DXL_ID_LIST[i]);
            if (ret) ret = _dxl.setOperatingMode(DXL_ID_LIST[i], mode);
        }
    }
    if (ret) ret = _dxl.torqueOn(BROADCAST_ID);
    return ret;
}

bool Motor::setPID(int id, int p, int i, int d) {
    bool ret = false;
    ret = _dxl.writeControlTableItem(ControlTableItem::POSITION_P_GAIN, id, p);
    if (ret) ret = _dxl.writeControlTableItem(ControlTableItem::POSITION_I_GAIN, id, i);
    if (ret) ret = _dxl.writeControlTableItem(ControlTableItem::POSITION_D_GAIN, id, d);
    return ret;
}

bool Motor::setVelocity(int id, int vel) {
    return _dxl.writeControlTableItem(ControlTableItem::PROFILE_VELOCITY, id, vel);
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

//Ref: https://github.com/ROBOTIS-GIT/Dynamixel2Arduino/blob/master/examples/advanced/sync_read_write_raw/sync_read_write_raw.ino
bool Motor::setGoalPositions(int val[]) {
    for (unsigned int i = 0; i < DXL_ID_CNT; i++) {
        sw_data[i].goal_position = val[i];
    }
    sw_infos.is_info_changed = true;

    if (_dxl.syncWrite(&sw_infos) == true) {
        _debug->println("[SyncWrite] Success");
    } else {
        sprintf(_msg, "[SyncWrite] Fail, ErrorCode: %d", (int)_dxl.getLastLibErrCode());
        _debug->print(_msg);
    }
    return true;
}

// Get present values(current, position)
int* Motor::getPresentValues() {
    uint8_t recv_cnt;
    static int results[Motor::DXL_ID_CNT * 2];  //change size to add velocity

    recv_cnt = _dxl.syncRead(&sr_infos);
    if (recv_cnt > 0) {
        sprintf(_msg, "[SyncRead] Success, Received ID Count: %d", recv_cnt);
        _debug->println(_msg);
        for (int i = 0; i < recv_cnt; i++) {
            sprintf(_msg, "  ID: %d, Err: %d\t Crr: %d \t Pos: %d", sr_infos.p_xels[i].id, sr_infos.p_xels[i].error, sr_data[i].present_current, (int)sr_data[i].present_position);
            _debug->println(_msg);
            results[i] = abs(sr_data[i].present_current);
            results[i + Motor::DXL_ID_CNT] = sr_data[i].present_position;
        }
    } else {
        sprintf(_msg, "[SyncRead] Fail, ErrorCode: %d", (int)_dxl.getLastLibErrCode());
        _debug->println(_msg);
    }
    return results;
}