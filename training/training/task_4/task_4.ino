#include <ros.h>  //library for ROS(compulsory)                 
#include <geometry_msgs/Twist.h> //message header geometry_msgs using twist
#include <geometry_msgs/Vector3Stamped.h> //message header geometry_msgs using Vector3Stamped

ros::NodeHandle nh;  //All node handle by ROS

//==========steering data declaration=====================//
float throttle,brake,steering;//subcribe declare
float target_info,speed_info,steering_info;//publish declare
//========================================================//

//==============Publish section===================//
geometry_msgs::Vector3Stamped joy_msg;
ros::Publisher joy_pub("/joystick_info", &joy_msg);
//================================================//

//================Subscribe section==================================//
void joystick(const geometry_msgs::Twist& msg){
  throttle = msg.linear.x;
  brake = msg.linear.y;
  steering=msg.linear.z;
}
ros::Subscriber<geometry_msgs::Twist> joy_sub("/cmd_vel", &joystick);
//===================================================================//

//==============motor declaration============//
//pin declaration
const int pwm = 5; //D3
const int dir = 4; //D3
//motor variable declartion
int pwmVal=0;
int dir_m=1;
//===========================================//

//===========encoder declaration=============//
//pin declaration
static int pinA = 2; //D5
static int pinB = 3;  //D6
//variable declaration
int pulsesChanged = 0;
int pulses=0;
int previous_pulse;
int pulse_per_period;
float rpm = 0;
float act_rpm=0;
float resolution = 3360; //refer data sheet
//==========================================//

//=====low pass filter declaration==========//
float RPM_Filt=0;
float RPMPrev=0;
//==========================================//

//======== Time variable declaration ========//
float time;
int period = 10;
float period_to_minute = (1000/period) * 60;
float deltaT,currT,prevT,eprev;
//==========================================//

//=========  PID declaration ===============//
float kp = 50;
float ki = 100;
float kd = 0;
float target_rpm=0;
float e=0;
float eintegral = 0;
float ediff=0;
float u=0;
//=========================================//

void setup() {
//Serial.begin(57600);
nh.getHardware()->setBaud(57600);

nh.initNode();                   //initiate all node 
nh.subscribe(joy_sub);           //info function take from master
nh.advertise(joy_pub);           //info function send to master

pinMode(pwm, OUTPUT);
pinMode(dir, OUTPUT);
pinMode(pinA, INPUT_PULLUP);
pinMode(pinB, INPUT_PULLUP);
attachInterrupt(digitalPinToInterrupt(pinA),A_CHANGE,CHANGE); 
attachInterrupt(digitalPinToInterrupt(pinB),B_CHANGE,CHANGE);

time = millis();

}

void loop() {

if (millis() > time + period)
  {
    time = millis();
    pulse_per_period = pulses - previous_pulse;
    previous_pulse = pulses;
    rpm = pulse_per_period * (period_to_minute / resolution);}

  
//========throttle conversion====//
//(from 0-1 to 0-30 rpm)
target_rpm=throttle*30;     
//or
//target_rpm=map(0,1,0,30); //<--- map method


//============low pass filter====================//
RPM_Filt = 0.8*RPM_Filt + 0.05*rpm + 0.05*RPMPrev;
RPMPrev = rpm;
//===============================================//

act_rpm=RPM_Filt;
//act_rpm=rpm;  
 
long currT = time;
deltaT = (currT-prevT)/1.0e3;
prevT = currT;
     
e=target_rpm-act_rpm;
eintegral = eintegral + e*deltaT;
ediff = (e-eprev)/(deltaT);

//u = kp*e + ki*eintegral+kd*ediff;
u = kp*e + ki*eintegral;

// Set the motor speed and direction
if (u==0){
    dir_m=0;
  }
else if (u>0){
    dir_m=1;
  }
else if (u<0){
    dir_m=-1;
  }
 
pwmVal = fabs(u);
  if(pwmVal > 255){
    pwmVal = 255;
  }
  
 setMotor();
 e=eprev;

//===massage info====//
target_info=target_rpm;
speed_info=act_rpm;
steering_info=steering;

//Publish massage using vector stamped header
joy_msg.vector.x = target_info;
joy_msg.vector.y = speed_info;
joy_msg.vector.z = steering_info;
joy_pub.publish(&joy_msg); 
nh.spinOnce();

}

void setMotor(){
  analogWrite(pwm,pwmVal);
  if(dir_m==1){
    digitalWrite(dir,HIGH);
  }
  else if(dir_m == -1){
    digitalWrite(dir,LOW);
  }
  else if(dir_m==0){
    digitalWrite(dir,HIGH);    
  }
}

//encoder interupt service routine
void A_CHANGE() {
  if ( digitalRead(pinB) == 0 ) {
    if ( digitalRead(pinA) == 0 ) {
      pulses--;
    } else {
      pulses++;
    }
  } else {
    if ( digitalRead(pinA) == 0 ) {
      pulses++;
    } else {
      // A rose, B is low
      pulses--;
    }
  }
  pulsesChanged = 1;
}


void B_CHANGE(){
  if ( digitalRead(pinA) == 0 ) {
    if ( digitalRead(pinB) == 0 ) {
      // B fell, A is low
      pulses++; // moving forward
    } else {
      // B rose, A is low
      pulses--; // moving reverse
    }
 } else {
    if ( digitalRead(pinB) == 0 ) {
      // B fell, A is high
      pulses--; // moving reverse
    } else {
      // B rose, A is high
      pulses++; // moving forward
    }
  }
  pulsesChanged = 1;
}
