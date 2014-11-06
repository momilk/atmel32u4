void setup()
{

  pinMode(13,OUTPUT);
  /* add setup code here */
  Serial.begin(9600);

}

void loop()
{
    digitalWrite(13,HIGH);
	delay(2000);
	 digitalWrite(13,LOW);
	 delay(2000);

  /* add main program code here */

}
