#include <Hand.h>
#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>

#define DEBUG true

Debug debug(&Serial, DEBUG);
Motor motor(&debug);
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