/*
 distinguish between long and short press on int0 button.
 Int0Flag = true each time int0 btn pressed.
 only when Int0Pressed=false (first pressed) the
 function corresponding to int0 btn press is operated.
*/


boolean Int0Pressed = false;
boolean Int0Flag = false;


void setup()
{ 
  pinMode(13,OUTPUT);
  pinMode(12,OUTPUT);
   pinMode(2,INPUT);
  digitalWrite(13,LOW);
  digitalWrite(12,LOW);
  Serial.begin(9600);
  //InterruptZeroInit();
  cli();
  EICRA |= (0<<ISC00)|(1<<ISC01);  // EICRA sets int0 for LOW level
  EIMSK |= (1<<INT0);
  sei();
}

void loop(){	
	if(Int0Flag ==true & Int0Pressed==false ) 
	{
		Int0Pressed=true;
		delay(5000);
		if (digitalRead(2)==LOW){
			Serial.println("Long press");
		}
		else{
			Serial.println("short press");
		}
	}
}
	
void InterruptZeroInit(){	
}

ISR(INT0_vect){
	Int0Flag=true;
}


 
