//motor pin
const int pwm = 5; //D3
const int dir = 4; //D4

//encoder pin
static int pinA = 2; //D5
static int pinB = 3;  //D6

//Encoder variable declaration
int pulsesChanged = 0;
int pulses=0;
float act_pos;
float pulse_to_degree=0.107142857142857; //(360/3360)
int resolution = 3360;

//motor variable declartion
int pwmVal=0;
int dir_m=1;

//Looping variable declaration
float time;
float deltaT,currT,prevT,eprev;

//PID
float kp = 120;
float ki = 100;
float kd = 0;
float e=0;
float eintegral = 0;
float ediff=0;
float u=0;

//target position initial
int target_pos=0;

void setup() {
Serial.begin(57600);

pinMode(pwm, OUTPUT);
pinMode(dir, OUTPUT);

pinMode(pinA, INPUT_PULLUP);
pinMode(pinB, INPUT_PULLUP);
attachInterrupt(digitalPinToInterrupt(pinA),A_CHANGE,CHANGE); 
attachInterrupt(digitalPinToInterrupt(pinB),B_CHANGE,CHANGE);


time = millis();

}

void loop() {

long currT = time;
deltaT = (currT-prevT)/1.0e3;
prevT = currT;

//PID
target_pos=-360;//in degree

act_pos=pulses*pulse_to_degree; 
e=target_pos-act_pos; //error
eintegral = eintegral + e*deltaT;
ediff = (e-eprev)/(deltaT);
//u = kp*e + ki*eintegral+kd*ediff;

u = kp*e+ ki*eintegral;

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

Serial.print(target_pos);
Serial.print("\t");
Serial.print(act_pos);
Serial.println(" ");

}

//encoder interupt routine
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
