#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <math.h>
#define PI 3.142857143

ros::NodeHandle nh;

//yaw data declaration
float V,brake,yaw;                 //subcribe declare
float target_info,speed_info,yaw_info;  //publish declare

//=======Publish section===========//
geometry_msgs::Twist joy_msg;
ros::Publisher joy_pub("/robot_state", &joy_msg);

//Subscribe massage
void commandVelocityCallback(const geometry_msgs::Twist& joystick);
ros::Subscriber<geometry_msgs::Twist> joy_sub("/cmd_vel", commandVelocityCallback);

//===motor driver declaration===//
//motor right pin
const int pwm_right = 10; //D3
const int dir_right = 34;


//motor left pin
const int pwm_left = 9;
const int dir_left = 32; //D3


//motor right variable
int pwmVal_right=0;
int dir_m_right=1;
float pwm_motor_right=0;

//motor left variable
int pwmVal_left=0;
int dir_m_left=1;
float pwm_motor_left=0;

//====robot dynamic declaration=====//
float VRd=0;
float VLd=0;
float max_speed=0.5 ;   //refere to robot max speed at 255 pwm
float max_yaw=1;

void setup() {
//Serial.begin(57600);
nh.getHardware()->setBaud(57600);

nh.initNode();                   //initiate all node 
nh.subscribe(joy_sub);           //info take from master
nh.advertise(joy_pub);           //info send to master

pinMode(pwm_right, OUTPUT);
pinMode(dir_right, OUTPUT);
pinMode(pwm_left, OUTPUT);
pinMode(dir_left, OUTPUT);
}

void loop() {

//inverse kinematic function
move_motor();
   
publishjoy_msg();


nh.spinOnce();

}


//========Subscribe section========//
void commandVelocityCallback(const geometry_msgs::Twist& joystick)
{
  V = joystick.linear.x;
  brake = joystick.linear.y;
  yaw=joystick.angular.z;
}

void publishjoy_msg(void)
{
joy_msg.linear.x = pwm_motor_right;
joy_msg.linear.y = pwm_motor_left;
joy_msg.linear.z = V;
joy_msg.angular.z = yaw;
joy_pub.publish(&joy_msg); 
}

void move_motor()
{
  VRd=(V*max_speed)-((yaw*max_yaw));  // VLd=(L*yaw)+V
  VLd=((yaw*max_yaw))+(V*max_speed); // VRd=(L*yaw)+V

pwm_motor_right=VRd*85;
pwm_motor_left=VLd*85;
  
 if (pwm_motor_right>0){
    analogWrite(pwm_right,pwm_motor_right);
    digitalWrite(dir_right,HIGH);
  }
  else if(pwm_motor_right<0){
  analogWrite(pwm_right,fabs(pwm_motor_right));
  digitalWrite(dir_right,LOW);
 }

 if (pwm_motor_left>0){
    analogWrite(pwm_left,pwm_motor_left);
    digitalWrite(dir_left,HIGH);
  }
  else if(pwm_motor_left<0){
  analogWrite(pwm_left,fabs(pwm_motor_left));
  digitalWrite(dir_left,LOW);
 } 
  
}
