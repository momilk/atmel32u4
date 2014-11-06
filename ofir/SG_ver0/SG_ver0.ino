//pressed button states
#define IDLE_STATE 0 //sleep mode
#define BLT_SEESION 1// transfer data in EEPROM to Android
#define NEW_SESSION 2 //first pressed with no side select if selected go to pre else wait for 5 minutes

//button0 press options
#define BTN0_LONG  10
#define BTN_SHORT 11


int current_state=IDLE_STATE; 
int btnPressTime;            //holds var for short or long press
boolean Int0_flag = false;   //int0 flag
boolean Int0_pressed = false;//int0 btn already pressed

void setup()
{
	Serial.begin(9600);
	cli();
	interruptInit();
	sei();
}

void loop()
{
	if(syncTime==false)
	{
		syncTimeFunction();
	}
	if(Int0Flag==true){
		 updateState();	
	}
}

ISR(INT0_vect){
	if(current_state==IDLE_STATE){
		btnPressTime=Int0BtnTime();// returns int value of short or long press
		if(btnPressTime==BTN0_LONG){
			current_state=BLT_SEESION; //long btn0 press go to Blt session.
		}
		else{
			current_state=NEW_SESSION; //short btn0 press go to New session.
		}
	}
	else{
		Int0Flag = true; //btn0 press and not idle state
	}
}

void interruptInit(){
	EICRA |= (0<<ISC00)|(1<<ISC01);  // EICRA sets int0 for falling ledge
    EIMSK |= (1<<INT0); //enable timer0
}

void updateState(){
	switch (current_state) {
		case IDLE_STATE:
		current_state=NEW_SESSION;
		break;
		case NURSERING_STATE:
		current_state=POST_MEASUREMENT;
		cli();
		TIMSK1 |= (0 << OCIE1A);//timer1 disable
		sei();
		prePostTimeError();
		current_state=IDLE;
		break;
		case POST_STATE:
		current_state=POST_MEASUREMENT
		break;
		default:
		// if nothing else matches, do the default
		// default is optional
	}