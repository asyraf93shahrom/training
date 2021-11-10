#include <ros.h>                 //compulsory to use this library when using ROS in arduino
#include <geometry_msgs/Twist.h> //data type massage geometry (linear xyz & angular xyz)
#include <geometry_msgs/Vector3Stamped.h>

ros::NodeHandle nh;  //All node info handle by ROS

float throttle,brake,steering;                 //subcribe declare
float throttle_info,brake_info,steering_info;  //publish declare

//=======Publish section===========//
geometry_msgs::Vector3Stamped joy_msg;
ros::Publisher joy_pub("/joystick_info", &joy_msg);


//========Subscribe section========//
void joystick(const geometry_msgs::Twist& msg){
  throttle = msg.linear.x;
  brake = msg.linear.y;
  steering=msg.linear.z;
}
//Subscribe massage
ros::Subscriber<geometry_msgs::Twist> joy_sub("/cmd_vel", &joystick);


void setup() {
//Serial.begin(57600); <-- dont use this line if using ROS in arduino
nh.getHardware()->setBaud(57600);
 
nh.initNode();                   //initiate all node 
nh.subscribe(joy_sub);           //info take from master
nh.advertise(joy_pub);           //info send to master

}

void loop() {
throttle_info=throttle;
brake_info=brake;
steering_info=steering;


joy_msg.vector.x = throttle_info;
joy_msg.vector.y = brake_info;
joy_msg.vector.z = steering_info;
joy_pub.publish(&joy_msg); 
nh.spinOnce();    

}
