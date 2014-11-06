int sensor_pin=A0;
int sensor_val=0;
//static const int array_length=10;
//int sensor_value[array_length];

void setup()
{
	Serial.begin(9600);
	//analogReference(INTERNAL); TBD change to external
	/* add setup code here */
}

//int i=0;
//int x=0;
void loop()
{
	//sensor_value[i % array_length]=analogRead(sensor_pin);
	sensor_val=analogRead(sensor_pin);
	Serial.println(sensor_val);
	delay(1000);
	//i++;
	//delay(1000);
	//x = analogRead(sensor_pin);
	//i++;
	//for (i=0;i<array_length;i++)
	//{
		//sensor_value[i]=i;//analogRead(sensor_pin);
		////Serial.println(sensor_value[i]);
		//Serial.println(i);
	//}
	//for (i=0;i<array_length;i++)
	//{
		//Serial.println(sensor_value[i]);
	//}
}

