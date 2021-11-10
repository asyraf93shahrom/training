#include <ros.h>                 //compulsory to use this library when using ROS in arduino
#include <std_msgs/Float32.h>    //data type massage Float32

ros::NodeHandle nh;  //All node info handle by ROS

float rpm;

//=======Publish section===========//
std_msgs::Float32 sensor_msg; 
ros::Publisher sensor_pub("/sensor_data", &sensor_msg);

void setup() {
//Serial.begin(57600); <-- dont use this line if using ROS in arduino
nh.getHardware()->setBaud(57600);

nh.initNode();            //initiate node (advertise/subscribe)
nh.advertise(sensor_pub); //info send to master
}

void loop() {

 
//Serial.print(rpm); <-- dont use serial print when using ROS
//Serial.println(" ");
rpm =10;
sensor_msg.data = rpm;
sensor_pub.publish(&sensor_msg);

nh.spinOnce();  //node handle spin (loop)  

}
