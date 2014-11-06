#include <SoftwareSerial.h>

int sensor_pin=A0;
int sensor_val=0;


SoftwareSerial bluetoothSerial(10, 11); // RX, TX
void setup()
{
	Serial.begin(9600);
	bluetoothSerial.begin(9600);
	//analogReference(INTERNAL); TBD change to external
}

//int i=0;
//int x=0;
void loop()
{
	//sensor_value[i % array_length]=analogRead(sensor_pin);
	sensor_val=analogRead(sensor_pin);
	Serial.println(sensor_val);
	bluetoothSerial.print(sensor_val);
	delay(1000);
}
