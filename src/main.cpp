#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <Hand.h>

#define DXL Serial2
#define DXL_DIR_PIN 2
#define DXL_PROTOCOL_VERSION 2.0
#define DXL_BAUDRATE 57600

#define DEBUG true

Dynamixel2Arduino dxl(DXL, DXL_DIR_PIN);
Debug debug(&Serial, DEBUG);
Motor motor(dxl, DXL_BAUDRATE, DXL_PROTOCOL_VERSION, &debug);
Hand hand(motor, &debug);

//ros::NodeHandle nh;

// void messageCb(const std_msgs::Empty& toggle_msg) {
//     digitalWrite(13, HIGH - digitalRead(13));  // blink the led
// }

//ros::Subscriber<std_msgs::Empty> sub("toggle_led", messageCb);

//std_msgs::String str_msg;
//ros::Publisher chatter("chatter", &str_msg);

//char hello[13] = "hello world!";

void setup() {
    Serial.begin(115200, SERIAL_8N1);
    pinMode(13, OUTPUT);
    //nh.initNode();
    //nh.advertise(chatter);
    //nh.subscribe(sub);

    motor.init();
    hand.init();
}

void loop() {
    //str_msg.data = hello;
    //chatter.publish(&str_msg);
    //nh.spinOnce();
    delay(500);
}