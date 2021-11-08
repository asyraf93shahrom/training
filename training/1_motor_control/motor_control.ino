const int pwm = PE13; //D3
const int In1 = PF14; //D4
const int In2 = PF15; //D2

int pwmVal=255;
int dir_m=1;

void setup() {

  pinMode(pwm, OUTPUT);
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);

}

void loop() {
 pwmVal=0;
 dir_m=-1;
 setMotor();

 
 
}

void setMotor(){
  analogWrite(pwm,pwmVal);
  if(dir_m==1){
    digitalWrite(In1,HIGH);
    digitalWrite(In2,LOW);
  }
  else if(dir_m == -1){
    digitalWrite(In1,LOW);
    digitalWrite(In2,HIGH);
  }
  else{
    digitalWrite(In1,LOW);
    digitalWrite(In2,LOW);    
  }
}
