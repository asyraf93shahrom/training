const int pwm = 5; //D3
const int dir = 4; //D4
static int pinA = 3; //D5
static int pinB = 2;  //D6

//Encoder variable declaration
int pulsesChanged = 0;
int pulses;
int previous_pulse;
int pulse_per_period;
float rpm = 0;
float resolution = 3000;

//motor variacle declartion
int pwmVal=0;
int dir_m=1;

//Looping variable declaration
float time;
int period = 10;
float period_to_minute = (1000/period) * 60;


void setup() {
Serial.begin(57600);

pinMode(pwm, OUTPUT);
pinMode(dir, OUTPUT);
pinMode(pinA, INPUT_PULLUP);
pinMode(pinB, INPUT_PULLUP);
attachInterrupt(1,A_CHANGE,CHANGE); 
attachInterrupt(0,B_CHANGE,CHANGE);

time = millis();

}

void loop() {

if (millis() > time + period)
  {
    time = millis();
    pulse_per_period = pulses - previous_pulse;
    previous_pulse = pulses;
    rpm = pulse_per_period * (period_to_minute / resolution);

    Serial.print(rpm);
    Serial.print("\t");
    Serial.print(pulses);
    Serial.print("\t");
    Serial.println(" ");

   }
   digitalWrite (dir,LOW
   );
   analogWrite(pwm,100);
 
 //setMotor();

}

//void setMotor(){
//  analogWrite(pwm,pwmVal);
//  if(dir_m==1){
//    digitalWrite(In1,HIGH);
//    digitalWrite(In2,LOW);
//  }
//  else if(dir_m == -1){
//    digitalWrite(In1,LOW);
//    digitalWrite(In2,HIGH);
//  }
//  else{
//    digitalWrite(In1,LOW);
//    digitalWrite(In2,LOW);    
//  }
//}


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
