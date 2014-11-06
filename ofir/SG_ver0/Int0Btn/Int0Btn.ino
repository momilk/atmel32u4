/*
 distinguish between long (3sec press) and short press on int0 button.
 Int0Flag = true each time int0 btn pressed.
 only when Int0Pressed=false (first pressed) the
 function corresponding to int0 btn press is operated.
*/
#define BTN0_LONG  10
#define BTN_SHORT 11
#define INT0_BTN 12

int Int0BtnTime(){
	if(Int0_flag ==true & Int0_pressed==false )
	{
		Int0_pressed=true;
		delay(3000); 
		if (digitalRead(INT0_BTN)==LOW){
			return BTN0_LONG;
		}
		else{
			return BTN_SHORT;
		}
	}
}
