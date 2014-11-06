boolean state=true;
int Timer1Cnt=0;
int timer1dis=0;
int timer1Continue=0;
/*
flash led pin13 timer1dis times, disable timer1 counter.
wait 10 seconds delay.
enable timer1 
and start over and over routine.
*/

void setup()
{ 
  pinMode(13,OUTPUT);
  Serial.begin(9600);
  // initialize timer, interrupt and variable
  timer1_init();
}

void loop()
{
	if(timer1dis==3){
		Serial.print("timer1 dis=3");
		//TIMSK1 |= (0 << OCIE1A);
		TCCR1B |= (1 << CS12) | (1 << CS11) | (1 << CS10);  //timer1 stop
		timer1dis=0;
		timer1Continue=1; 
	}
	if (timer1Continue==1)
	{
		delay(10000);
		timer1Continue=0;
		timer1_init();
		//TCCR1B |= (1 << CS12) | (0 << CS11) | (1 << CS10);
		Serial.print("timer1 enable");
	}
}

// initialize timer, interrupt and variable
void timer1_init(){
	cli();//stop interrupts
	//set timer1 interrupt at 0.25Hz, interupt in 4sec gap.
	TCCR1A = 0;// set entire TCCR1A register to 0
	TCCR1B = 0;// same for TCCR1B
	TCNT1  = 0;//initialize counter value to 0
	// set compare match register for 0.25hz 4sec increments
	OCR1A = 62499;// = (16*10^6) / (0.25*1024) - 1 (must be <65536)
	// turn on CTC mode - Clear Timer on Compare match (CTC) mode
	TCCR1B |= (1 << WGM12);
	// Set CS10 and CS12 bits for 1024 prescaler
	TCCR1B |= (1 << CS12) | (1 << CS10);
	// enable timer compare interrupt ( the Timer/Counter1 Output Compare A Match interrupt is enabled)
	TIMSK1 |= (1 << OCIE1A);
	sei();//allow interrupts
}

 ISR(TIMER1_COMPA_vect){
    //3minCounter++;
    //20minCounter++;
    if(Timer1Cnt==0){
	    Timer1Cnt=0;
		if (state==true)
		{
			digitalWrite(13,HIGH);
			state=false;
		}
		else{
			digitalWrite(13,LOW);
			state=true;
		}
	}
	//Timer1Cnt++;
	timer1dis++;
    //if(3minCounter==45){
		//timer3MinPass=true;//3minpass
	    //3minCounter=0;
    //}
     //if(20minCounter==300){
		 //timer20MinPass=true;//20minpass
	     //20minCounter=0;
	 //}
  }