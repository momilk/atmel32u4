/*
  AnalogReadSerial
 Reads an analog input on pin 0, prints the result to the serial monitor.
 Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.
 
 This example code is in the public domain.
 */
int ledPin=13;
int state =0;
int flag=0;


// the setup routine runs once when you press reset:
void setup(){
  //Led pin output Low
  pinMode(ledPin,OUTPUT);
  digitalWrite(ledPin,LOW);
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
}

// the loop routine runs over and over again forever:
void loop(){
  if(Serial.available()>0){
    state = Serial.read();
    flag=0;
  }
  if(state=='0'){
    digitalWrite(ledPin,LOW);
    if(flag==0){
      Serial.println("Led:Off");
      flag=1;
    }
  }
  else if(state=='1'){
    digitalWrite(ledPin,HIGH);
    if(flag==0){
      Serial.println("Led:On");
      flag=1;
    }
  }
}

