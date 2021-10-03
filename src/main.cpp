#include <Hand.h>
#include <Thread.h>
#include <ThreadController.h>
#include <gripper_ntlab_controller/JointPosition.h>
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

Motor motor(&debug);
Hand hand(motor, &debug);

ros::NodeHandle nh;

const int DOF = 5;
const char robot[8] = "gripper";

float pos[DOF], vel[DOF], eff[DOF];

int mode;
int position[DOF] = {0};

int* values;
char* joint_name[DOF] = {"l_base_hand_r_position_controller",
                         "r_base_hand_r_position_controller",
                         "l_hand_rod_a_position_controller",
                         "r_hand_rod_a_position_controller",
                         "l_finger_roll_position_controller"};

void jointSubs(const gripper_ntlab_controller::JointPosition& sub_msg) {
    mode = sub_msg.mode;
    for (unsigned int i = 0; i < DOF; i++) {
        position[i] = (int)sub_msg.position[i];
    }
}

ros::Subscriber<gripper_ntlab_controller::JointPosition> sub("gripper_ntlab/SetPosition", jointSubs);

sensor_msgs::JointState pub_msg;
ros::Publisher pub("gripper_ntlab/JointState", &pub_msg);

sensor_msgs::JointState setMsg() {
    sensor_msgs::JointState msg;
    values = hand.getPresentValue();
    for (int i = 0; i < motor.DXL_ID_CNT; i++) {
        pos[i] = *(values + i + motor.DXL_ID_CNT);
        vel[i] = *(values + i + motor.DXL_ID_CNT * 2);
        eff[i] = *(values + i);
    }

    //Finger Servo
    pos[DOF - 1] = 0;
    vel[DOF - 1] = 1;
    eff[DOF - 1] = mode;

    msg.header.frame_id = robot;
    msg.header.stamp = nh.now();

    msg.name_length = DOF;
    msg.velocity_length = DOF;
    msg.position_length = DOF;
    msg.effort_length = DOF;

    msg.name = joint_name;
    msg.position = pos;
    msg.velocity = vel;
    msg.effort = eff;
    return msg;
}

void motorControlCallback() {
    if (mode == 72) {
        hand.movePositionRelativePrecision(position);
    } else if (mode == 96) {
        hand.movePositionRelative(position);
    } else if (mode == 112) {
        hand.movePosition(position);
    } else if (mode == 124) {
        hand.calibratePositionBulk();
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

    motorControlThread->onRun(motorControlCallback);
    motorControlThread->setInterval(500);

    publishThread->onRun(publishCallback);
    publishThread->setInterval(100);

    threadController.add(motorControlThread);
    threadController.add(publishThread);
}

void loop() {
#if DEBUG
    testCase1();
#else
    threadController.run();
    hand.updatePresentValue();
    nh.spinOnce();
#endif
}

bool testCase1() {
    int value[DOF];
    value[0] = 110;
    value[1] = -110;
    value[2] = 110;
    value[3] = -110;
    value[DOF - 1] = 0;
    bool move = hand.movePositionRelative(value);
    delay(5000);

    value[0] = -100;
    value[1] = 100;
    value[2] = -100;
    value[3] = 100;
    value[DOF - 1] = 0;
    move = hand.movePositionRelative(value);
    delay(5000);

    value[0] = 200;
    value[1] = -200;
    value[2] = 200;
    value[3] = -200;
    value[DOF - 1] = 0;
    move = hand.movePositionRelative(value);
    delay(5000);

    value[0] = -100;
    value[1] = 100;
    value[2] = -100;
    value[3] = 100;
    value[DOF - 1] = 0;
    move = hand.movePositionRelative(value);
    delay(5000);

    value[0] = 200;
    value[1] = -200;
    value[2] = 200;
    value[3] = -200;
    value[DOF - 1] = 0;
    move = hand.movePositionRelative(value);
    delay(50000);

    return move;
}