const int pwm = 5; //D3
const int dir = 4; //D3
static int pinA = 2; //D5
static int pinB = 3;  //D6

//Encoder variable declaration
int pulsesChanged = 0;
int pulses=0;
int previous_pulse;
int pulse_per_period;
float rpm = 0;
float act_rpm=0;
float resolution = 3360;

//motor variacle declartion
int pwmVal=0;
int dir_m=1;

//Looping variable declaration
float time;
int period = 10;
float period_to_minute = (1000/period) * 60;
float deltaT,currT,prevT,eprev;

//PID
float kp = 50;
float ki = 100;
//float kd = 0;
float e=0;
float dr_rpm=0;
float eintegral = 0;
float ediff=0;
float u=0;

//filter
float RPM_Filt=0;
float RPMPrev=0;

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

if (millis() > time + period)
  {
    time = millis();
    pulse_per_period = pulses - previous_pulse;
    previous_pulse = pulses;
    rpm = pulse_per_period * (period_to_minute / resolution);
    
   }
//data low pass filter
   RPM_Filt = 0.8*RPM_Filt + 0.05*rpm + 0.05*RPMPrev;
  RPMPrev = rpm;

  act_rpm=RPM_Filt;
  //act_rpm=rpm;
   
long currT = time;
deltaT = (currT-prevT)/1.0e3;
prevT = currT;
  
dr_rpm=0;
//dr_rpm = 30*(sin(currT/1e3));   
e=dr_rpm-act_rpm;
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

 Serial.print(act_rpm);
 Serial.print("\t");
 Serial.print(dr_rpm);
 Serial.print("\t");
// Serial.print(u);
//  Serial.print("\t");
// Serial.print(dir_m);
 Serial.println(" ");

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