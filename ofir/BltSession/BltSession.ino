#include <SoftwareSerial.h>// import the serial library

int ledpin=13; // led on D13 will show blink on / off
char BltDataIn; // the data given from android
char* myStrings[]={"str 1", "str 2","str 3", "str 4","str 5"};

boolean cont=false;
SoftwareSerial Genotronex(10, 11); // RX, TX 


void setup() {
	// put your setup code here, to run once:
	Serial.begin(9600);     // Serial serial begin 9600 baud rate
	Genotronex.begin(9600); // Blt serial begin 9600 baud rate
//	pinMode(ledpin,OUTPUT);  
	//pinMode(9,OUTPUT);
	//digitalWrite(9,HIGH);
}

int i;

void loop() {
	//get data from android to arduino serial monitor
	char recvChar;
	//if (Serial.available()>0) {
	//	recvChar =  Serial.read();
	//	Genotronex.print(recvChar);
	//}

        if (Genotronex.available()>0) {
		recvChar =  Genotronex.read();
		Serial.print(recvChar);
	}

	//send data from arduino to android, sends 0,1,2 and "str1" "str2"
	/*
	if(cont==false){
		for (i=0;i<3;i++)
		{
			Genotronex.println(i);
			delay(1000);
		}
	for ( i = 0; i <2; i++){
		Genotronex.println(myStrings[i]);
		delay(1000);
	}
	}
	cont=true;
	*/
//	getBltData();
}

void getBltData() {
	char recvChar;
	if (Genotronex.available()>0) {
		recvChar =  Genotronex.read();
	    Serial.print(recvChar);
	}
}
