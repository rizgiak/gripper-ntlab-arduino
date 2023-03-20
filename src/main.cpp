#include <Hand.h>
#include <Thread.h>
#include <ThreadController.h>
#include <gripper_ntlab_msgs/JointPosition.h>
#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

#define DEBUG false

#if DEBUG
char msg[100];
Debug debug(&Serial, DEBUG);
#else
Debug debug(NULL, DEBUG);
#endif

ThreadController threadController = ThreadController();
Thread* motorControlThread = new Thread();
Thread* publishThread = new Thread();

// define global function
bool testCase1();
bool testCase2();

Motor motor(&debug);
Hand hand(motor, &debug);

ros::NodeHandle nh;

const char robot[8] = "gripper";

float pos[motor.DOF], vel[motor.DOF], eff[motor.DOF];

int mode;
int position[motor.DOF] = {0};

int* values;
int* calib_values;

char* joint_name[motor.DOF] = {"l_base_hand_s",
                               "r_base_hand_s",
                               "l_hand_rod_a",
                               "r_hand_rod_a",
                               "l_finger_roll"};

void jointSubs(const gripper_ntlab_msgs::JointPosition& sub_msg) {
    mode = sub_msg.mode;

    for (unsigned int i = 0; i < motor.DOF; i++) {
        if (mode == 101) {
            position[i] = (int)sub_msg.position[i] + *(calib_values + i);
        } else {
            position[i] = (int)sub_msg.position[i];
        }
    }
}

// based on Dynamixel performance graph
// https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/
float currentToTorque(float val){
    return (float)(1.731448764 * (val / 1000)); //- 0.12614841
}

ros::Subscriber<gripper_ntlab_msgs::JointPosition> sub("gripper_ntlab/set_position", jointSubs);

sensor_msgs::JointState pub_msg;
ros::Publisher pub("gripper_ntlab/joint_states", &pub_msg);

sensor_msgs::JointState setMsg() {
    sensor_msgs::JointState msg;
    values = hand.getPresentValue();
    for (int i = 0; i < (motor.DOF); i++) {                          // + 1 add servo position
        pos[i] = *(values + i + (motor.DOF)) - *(calib_values + i);  // 0 is absolute as calibration values
        vel[i] = *(values + i + (motor.DOF) * 2);
        eff[i] = currentToTorque(*(values + i));
        // eff[i] = 1;
    }

    msg.header.frame_id = robot;
    msg.header.stamp = nh.now();

    msg.name_length = motor.DOF;
    msg.velocity_length = motor.DOF;
    msg.position_length = motor.DOF;
    msg.effort_length = motor.DOF;

    msg.name = joint_name;
    msg.position = pos;
    msg.velocity = vel;
    msg.effort = eff;
    return msg;
}

void motorControlCallback() {
    if (mode == 72) {
        hand.calibratePositionBulk();
    } else if (mode == 101) {
        hand.movePosition(position);
    } else if (mode == 112) {
        hand.movePositionRelative(position);
    } else if (mode == 124) {
        hand.movePositionRelativePrecision(position);
        mode = 0;
    }
}

void publishCallback() {
    pub_msg = setMsg();
    pub.publish(&pub_msg);
}

void setup() {
#if DEBUG
    Serial.begin(115200, SERIAL_8N1);
#endif
    pinMode(13, OUTPUT);
#if DEBUG == false
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.advertise(pub);
    nh.subscribe(sub);
#endif
    motor.init();
    hand.init();
    calib_values = hand.getCalibrationValue();

    motorControlThread->onRun(motorControlCallback);
    motorControlThread->setInterval(50);

    publishThread->onRun(publishCallback);
    publishThread->setInterval(100);

    threadController.add(motorControlThread);
    threadController.add(publishThread);
}

void loop() {
#if DEBUG
    //testCase2();
#else
    threadController.run();
    hand.updatePresentValue();
    nh.spinOnce();
#endif
}

#if DEBUG
bool testCase1() {
    int value[motor.DOF];
    value[0] = 110;
    value[1] = -110;
    value[2] = 110;
    value[3] = -110;
    value[motor.DOF - 1] = 0;
    bool move = hand.movePositionRelative(value);
    delay(5000);

    value[0] = -100;
    value[1] = 100;
    value[2] = -100;
    value[3] = 100;
    value[motor.DOF - 1] = 0;
    move = hand.movePositionRelative(value);
    delay(5000);

    value[0] = 200;
    value[1] = -200;
    value[2] = 200;
    value[3] = -200;
    value[motor.DOF - 1] = 0;
    move = hand.movePositionRelative(value);
    delay(5000);

    value[0] = -100;
    value[1] = 100;
    value[2] = -100;
    value[3] = 100;
    value[motor.DOF - 1] = 0;
    move = hand.movePositionRelative(value);
    delay(5000);

    value[0] = 200;
    value[1] = -200;
    value[2] = 200;
    value[3] = -200;
    value[motor.DOF - 1] = 0;
    move = hand.movePositionRelative(value);
    delay(50000);

    return move;
}

bool testCase2() {
    // int value[motor.DOF];
    // value[0] = 110;
    // value[1] = -110;
    // value[2] = 110;
    // value[3] = -110;
    // value[motor.DOF - 1] = 500;
    // hand.movePositionRelative(value);
    motor.move(500);
    // hand.updatePresentValue();
    // values = hand.getPresentValue();
    // for (int i = 0; i < motor.DXL_ID_CNT; i++) {
    //     sprintf(msg, "j%d: %d, ", i, *(values + i + motor.DXL_ID_CNT));
    //     debug.print(msg);
    // }
    // debug.println("");
    float a = motor.getPresentPosition(motor.DXL_ID_CNT) * 1.0;
    sprintf(msg, "testCase2, get, getPresentPosition: %1.2f", a);
    debug.println(msg);

    delay(1000);

    motor.move(1364);
    a = motor.getPresentPosition(motor.DXL_ID_CNT) * 1.0;
    sprintf(msg, "testCase2, get, getPresentPosition: %1.2f", a);
    debug.println(msg);
    delay(1000);
}
#endif