#include <Hand.h>
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
Motor motor(&debug);
Hand hand(motor, &debug);

ros::NodeHandle nh;

const int DOF = 5;
const char robot[8] = "gripper";

float pos[DOF], vel[DOF], eff[DOF];

int* values;
char* joint_name[DOF] = {"l_base_hand_r_position_controller",
                         "r_base_hand_r_position_controller",
                         "l_hand_rod_a_position_controller",
                         "r_hand_rod_a_position_controller",
                         "l_finger_roll_position_controller"};

void jointSubs(const sensor_msgs::JointState& sub_msg) {
    if (sub_msg.header.frame_id == "set") {
        int value[4];
        for (int i; i < motor.DXL_ID_CNT; i++) {
            value[i] = sub_msg.position[i];
        }
        hand.movePositionRelative(value);
    }
}

ros::Subscriber<sensor_msgs::JointState> sub("gripper_ntlab/SetPosition", jointSubs);

sensor_msgs::JointState pub_msg;
ros::Publisher pub("gripper_ntlab/JointState", &pub_msg);

sensor_msgs::JointState setMsg() {
    sensor_msgs::JointState msg;
    values = hand.getCalibrationValue();
    for (int i = 0; i < motor.DXL_ID_CNT; i++) {
        pos[i] = *(values + i);
        vel[i] = 1;
        eff[i] = 1;
    }

    //Finger Servo
    pos[DOF - 1] = 0;
    vel[DOF - 1] = 1;
    eff[DOF - 1] = 1;

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

void setup() {
#if DEBUG
    Serial.begin(115200, SERIAL_8N1);
#endif
    pinMode(13, OUTPUT);
#if DEBUG == false
    nh.initNode();
    nh.advertise(pub);
    nh.subscribe(sub);
#endif
    motor.init();
    hand.init();
}

void loop() {
#if DEBUG
    values = hand.getCalibrationValue();
    for (int i = 0; i < motor.DXL_ID_CNT; i++) {
        sprintf(msg, "j%d: %d, ", i, *(values + i));
        debug.print(msg);
    }
    debug.println("");
#else
    pub_msg = setMsg();
    pub.publish(&pub_msg);
    nh.spinOnce();
    delay(10);
#endif
}