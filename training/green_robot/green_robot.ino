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

//motor left variable
int pwmVal_left=0;
int dir_m_left=1;

//===encoder pin declaration===//
//encoder right
static int pinA = 18; //D5
static int pinB = 19;  //D6
//encoder left
static int pinC =2;
static int pinD =3;

//====Encoder variable declaration====//
float resolution = 2000; //refer data sheet

//encoder right
int pulsesChanged_right = 0;
int pulses_right=0;
int previous_pulse_right;
int pulse_per_period_right;
float rpm_right = 0;
float actual_speed_right=0;

//encoder left
int pulsesChanged_left = 0;
int pulses_left=0;
int previous_pulse_left;
int pulse_per_period_left;
float rpm_left = 0;
float actual_speed_left=0;

//====robot dynamic declaration=====//
float r_right=0.25;  //right tire radius in (m)
float r_left=0.25;   //left tire radius in (m)
float Vr;
float Vl;
float L=1;        // length between rear tire
float V_robot,yaw_robot;
float max_speed=1.5 ;   //refere to robot max speed at 255 pwm
float max_yaw=0.75;

//time Looping variable declaration
float time;
int period = 10;
float period_to_minute = (1000/period) * 60;
float deltaT,currT,prevT,eprev_right,eprev_left;

//======PID declaration=======//
//motor right
float kp_right = 200;
float ki_right = 1000;
float kd_right = 0;
float e_right=0;
float desired_speed_right=0;
float eintegral_right = 0;
float ediff_right=0;
float u_right=0;

//motor left
float kp_left = 200;
float ki_left= 1000;
float kd_left = 0;
float e_left=0;
float desired_speed_left=0;
float eintegral_left = 0;
float ediff_left=0;
float u_left=0;
//================================//

//=====filter declaration======//
//encoder right filter
float RPM_Filt_right=0;
float RPMPrev_right=0;

//encoder left filter
float RPM_Filt_left=0;
float RPMPrev_left=0;

//=====================================//

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
pinMode(pinA, INPUT_PULLUP);
pinMode(pinB, INPUT_PULLUP);
pinMode(pinC, INPUT_PULLUP);
pinMode(pinD, INPUT_PULLUP);
attachInterrupt(digitalPinToInterrupt(pinA),A_CHANGE,CHANGE); 
attachInterrupt(digitalPinToInterrupt(pinB),B_CHANGE,CHANGE);
attachInterrupt(digitalPinToInterrupt(pinC),C_CHANGE,CHANGE); 
attachInterrupt(digitalPinToInterrupt(pinD),D_CHANGE,CHANGE);

time = millis();

}

void loop() {
getencoder();
   
//data low pass filter
  RPM_Filt_right = 0.8*RPM_Filt_right + 0.05*rpm_right + 0.05*RPMPrev_right;
  RPMPrev_right = rpm_right;

  RPM_Filt_left = 0.8*RPM_Filt_left + 0.05*rpm_left + 0.05*RPMPrev_left;
  RPMPrev_left = rpm_left;
  
//If non filter speed uncomment this line
//  actual_speed_right=rpm_right*(2*PI*r_right/60);
//  actual_speed_left=rpm_left*(2*PI*r_left/60);

//filter speed using LPF in m/s
  actual_speed_right=RPM_Filt_right*(2*PI*r_right/60);
  actual_speed_left=RPM_Filt_left*(2*PI*r_left/60);

//inverse kinematic function
inverseDifferential();
   
long currT = time;
deltaT = (currT-prevT)/1.0e3;
prevT = currT;

/******************************************************
 * Right Motor PID
 ******************************************************/
e_right=desired_speed_right-actual_speed_right;
eintegral_right = eintegral_right + e_right*deltaT;
ediff_right = (e_right-eprev_right)/(deltaT);

//u = kp*e + ki*eintegral+kd*ediff;
u_right = kp_right*e_right + ki_right*eintegral_right;

/******************************************************
 * Left Motor PID
 ******************************************************/
e_left=desired_speed_left-actual_speed_left;
eintegral_left = eintegral_left + e_right*deltaT;
ediff_left = (e_left-eprev_left)/(deltaT);

//u = kp*e + ki*eintegral+kd*ediff;
u_left = kp_left*e_left + ki_left*eintegral_left;


/******************************************************
 * Right Motor speed and direction
 ******************************************************/
  if (u_right==0){
    dir_m_right=0;
  }
  else if (u_right>0){
    dir_m_right=1;
  }
  else if (u_right<0){
    dir_m_right=-1;
  }
 
  pwmVal_right = fabs(u_right);
  if(pwmVal_right > 255){
    pwmVal_right = 255;
  }


/******************************************************
 * Left Motor speed and direction
 ******************************************************/
   if (u_left==0){
    dir_m_left=0;
  }
  else if (u_left>0){
    dir_m_left=1;
  }
  else if (u_left<0){
    dir_m_left=-1;
  }
 
  pwmVal_left = fabs(u_left);
  if(pwmVal_left > 255){
    pwmVal_left = 255;
  }
  
 setMotor_right();
 setMotor_left();
 e_right=eprev_right;
 e_left=eprev_left;

Differential_eqn();
publishjoy_msg();


nh.spinOnce();

}

void getencoder(){
  if (millis() > time + period)
  {
    time = millis();
    pulse_per_period_right = pulses_right - previous_pulse_right;
    previous_pulse_right = pulses_right;
    rpm_right = pulse_per_period_right * (period_to_minute / resolution);
    
    pulse_per_period_left = pulses_left - previous_pulse_left;
    previous_pulse_left = pulses_left;
    rpm_left = pulse_per_period_left * (period_to_minute / resolution);
    
   }
}
void setMotor_right()
{
  analogWrite(pwm_right,pwmVal_right);
  if(dir_m_right==1){
    digitalWrite(dir_right,HIGH);
  }
  else if(dir_m_right == -1){
    digitalWrite(dir_right,LOW);
  }
  else if(dir_m_right==0){
    digitalWrite(dir_right,HIGH);    
  }
}
void setMotor_left()
{
  analogWrite(pwm_left,pwmVal_left);
  if(dir_m_left==1){
    digitalWrite(dir_left,HIGH);
  }
  else if(dir_m_left == -1){
    digitalWrite(dir_left,LOW);
  }
  else if(dir_m_left==0){
    digitalWrite(dir_left,HIGH);    
  }
}


//encoder interupt routine

void A_CHANGE() {
  if ( digitalRead(pinB) == 0 ) {
    if ( digitalRead(pinA) == 0 ) {
      pulses_right--;
    } else {
      pulses_right++;
    }
  } else {
    if ( digitalRead(pinA) == 0 ) {
      pulses_right++;
    } else {
      // A rose, B is low
      pulses_right--;
    }
  }
  pulsesChanged_right = 1;
}


void B_CHANGE(){
  if ( digitalRead(pinA) == 0 ) {
    if ( digitalRead(pinB) == 0 ) {
      // B fell, A is low
      pulses_right++; // moving forward
    } else {
      // B rose, A is low
      pulses_right--; // moving reverse
    }
 } else {
    if ( digitalRead(pinB) == 0 ) {
      // B fell, A is high
      pulses_right--; // moving reverse
    } else {
      // B rose, A is high
      pulses_right++; // moving forward
    }
  }
  pulsesChanged_right = 1;
}
void C_CHANGE() {
  if ( digitalRead(pinD) == 0 ) {
    if ( digitalRead(pinC) == 0 ) {
      pulses_left--;
    } else {
      pulses_left++;
    }
  } else {
    if ( digitalRead(pinC) == 0 ) {
      pulses_left++;
    } else {
      // A rose, B is low
      pulses_left--;
    }
  }
  pulsesChanged_left = 1;
}


void D_CHANGE(){
  if ( digitalRead(pinC) == 0 ) {
    if ( digitalRead(pinD) == 0 ) {
      // B fell, A is low
      pulses_left++; // moving forward
    } else {
      // B rose, A is low
      pulses_left--; // moving reverse
    }
 } else {
    if ( digitalRead(pinD) == 0 ) {
      // B fell, A is high
      pulses_left--; // moving reverse
    } else {
      // B rose, A is high
      pulses_left++; // moving forward
    }
  }
  pulsesChanged_left = 1;
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
joy_msg.linear.x = desired_speed_right;
joy_msg.linear.y = actual_speed_right;
joy_msg.linear.z = V_robot;
joy_msg.angular.x = desired_speed_left;
joy_msg.angular.y = actual_speed_left;
joy_msg.angular.z = yaw_robot;
joy_pub.publish(&joy_msg); 
}


void inverseDifferential()
{
  desired_speed_right=(V*max_speed)-(L*(yaw*max_yaw)); // VRd=(L*yaw)+V
  desired_speed_left=(L*(yaw*max_yaw))+(V*max_speed);  // VLd=(L*yaw)+V
}

void Differential_eqn(){
  V_robot=(actual_speed_right +actual_speed_left)/2;     //V=(Vr+Vl)/2;
  yaw_robot=(actual_speed_left-actual_speed_right)/(2*L);//yaw=Vl-vr/2L
}
