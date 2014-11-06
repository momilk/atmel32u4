
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))

int ledPin = 13;                 // LED connected to digital pin 13
volatile  boolean state=false;
volatile  boolean state1=false;
volatile long unsigned eventTime;
volatile long unsigned previousEventTime;
volatile long unsigned timeSinceLastEvent;
volatile byte portBstatus;
volatile byte eventFlag;

void setup()
{
  InitialiseInterrupt();
   Serial.begin(9600);  // start serial for output
   pinMode(ledPin, OUTPUT); 
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
//  if(eventFlag==1 && (eventTime-previousEventTime>100)){
//     timeSinceLastEvent=eventTime;
//     previousEventTime=eventTime;
//     Serial.print( timeSinceLastEvent);
//     Serial.println("ms");
     Serial.println(portBstatus,BIN);
     eventFlag=0; 
//}
//    Serial.print(state1);
//  //It appears that delay is needed in order not to clog the port
//  delay(2500);
}


ISR(INT6_vect) {
  // interrupt code goes here
  state=!state; // toggle running variable
  digitalWrite(ledPin, state);
  }
  
void InitialiseInterrupt(){
  cli();		// switch interrupts off while messing with their settings  
 sbi(PCICR,PCIE0);// Enable PCINTo interrupt
  sbi(PCMSK0,PCINT4);// Enable PCINT4 interrupt      
  sei(); // I-bit in the Status Register (SREG) is set (one)
}
 ISR(PCINTo_vect) {    // Interrupt service routine. Every single PCINT8..14 (=ADC0..5) change  // will generate an interrupt: but this will always be the same interrupt routine
 portBstatus=PINB;
 eventTime-millis();
 eventFlag=1;
 
// state1 =!state1; // toggle running variable
// digitalWrite(6, HIGH);
}
