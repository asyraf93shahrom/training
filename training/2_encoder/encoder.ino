static int pinA = PE11; //D5
static int pinB = PE9;  //D6

//Encoder variable declaration
int pulsesChanged = 0;
int pulses;
int previous_pulse;
int pulse_per_period;

//Looping variable declaration
float time;
int period = 10;

//Velocity & RPM variable declaration
float rpm = 0;
float resolution = 2000;
float period_to_minute = (1000/period) * 60;

void setup() {
Serial.begin(57600);

 pinMode(pinA, INPUT_PULLUP);
 pinMode(pinB, INPUT_PULLUP);
 attachInterrupt(pinA,A_CHANGE,CHANGE); 
 attachInterrupt(pinB,B_CHANGE,CHANGE);


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
//    Serial.print(pulses);
//    Serial.print("\t");
    Serial.println(" ");


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
