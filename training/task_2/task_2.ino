#include <ros.h>                 //compulsory to use this library when using ROS in arduino
#include <std_msgs/Float32.h>    //data type massage Float32

ros::NodeHandle nh;  //All node info handle by ROS

//======Encoder parameters==//

//Encoder pin declare
static int pinA = 2; //D2
static int pinB = 3;  //D3

//Encoder variable declaration
int pulsesChanged = 0;
int pulses;
int previous_pulse;
int pulse_per_period;
float rpm = 0;
float resolution = 3360;

//=======time Looping=========//
float time;
int period = 10;
float period_to_minute = (1000/period) * 60;

//=======Publish section===========//
std_msgs::Float32 speed_msg; 
ros::Publisher speed_pub("/encoder_data", &speed_msg);

void setup() {
//Serial.begin(57600); <-- dont use this line if using ROS in arduino
nh.getHardware()->setBaud(57600);

pinMode(pinA, INPUT_PULLUP);
pinMode(pinB, INPUT_PULLUP);
attachInterrupt(digitalPinToInterrupt(pinA),A_CHANGE,CHANGE); 
attachInterrupt(digitalPinToInterrupt(pinB),B_CHANGE,CHANGE);

time = millis(); 

nh.initNode();           //initiate node (advertise/subscribe)
nh.advertise(speed_pub); //info send to master
}

void loop() {

if (millis() > time + period)
  {
    time = millis();
    pulse_per_period = pulses - previous_pulse;
    previous_pulse = pulses;
    rpm = pulse_per_period * (period_to_minute / resolution);
  }
  
//Serial.print(rpm); <-- dont use serial print when using ROS
//Serial.println(" ");

speed_msg.data = rpm;
speed_pub.publish(&speed_msg);

nh.spinOnce();  //node handle spin (loop)  

}


//encoder interupt routine 
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
