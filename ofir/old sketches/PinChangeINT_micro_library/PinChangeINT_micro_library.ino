#include <PinChangeInt.h>
#include <PinChangeIntConfig.h>

 int ledPin = 13;                 // LED connected to digital pin 13
volatile  boolean state=false;
volatile  boolean state1=false;

void setup()
{
  PCintPort::attachInterrupt(0, change,FALLING); // attach a PinChange Interrupt to our pin on the rising edge
  PCintPort::attachInterrupt(28, change,FALLING); // attach a PinChange Interrupt to our pin on the rising edge
  PCintPort::attachInterrupt(4, change,FALLING); // attach a PinChange Interrupt to our pin on the rising edge
   EICRB |= (0<<ISC60)|(1<<ISC61); // sets the interrupt type for EICRB (INT6). 
                              // EICRA sets interrupt type for INT0...3

/*
ISCn0  ISCn1   Where n is the interrupt. 0 for 0, etc
  0      0   Triggers on low level
  1      0   Triggers on edge
  0      1   Triggers on falling edge
  1      1   Triggers on rising edge
*/

EIMSK |= (1<<INT6); // activates the interrupt. 6 for 6, etc
}

void loop(){
}

ISR(INT6_vect) {
  // interrupt code goes here
  state=!state; // toggle running variable
  digitalWrite(ledPin, state);
  }
  
 void change() {
  // interrupt code goes here
  state1=!state1; // toggle running variable
  digitalWrite(ledPin, HIGH);
  }

