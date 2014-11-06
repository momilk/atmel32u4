/* 
 * TimeSerial.pde
 * example code illustrating Time library set through serial port messages.
 *
 * Messages consist of the letter T followed by ten digit time (as seconds since Jan 1 1970)
 * you can send the text on the next line using Serial Monitor to set the clock to noon Jan 1 2013
 T1357041600  
 *
 * A Processing example sketch to automatically send the messages is inclided in the download
 * On Linux, you can use "date +T%s > /dev/ttyACM0" (UTC time zone)
 */ 
 
#include <Time.h>  
#include <avr/sleep.h>
#include <avr/power.h>  

#define TIME_HEADER  "T"   // Header tag for serial time sync message
#define TIME_REQUEST  7    // ASCII bell character requests a time sync message 

boolean state=false;
void setup(){
  Serial.begin(9600);
  while (!Serial) ; // Needed for Leonardo only
  pinMode(13, OUTPUT);
  setSyncProvider( requestSync);  //set function to call when sync required
  Serial.println("Waiting for sync message");
  pinMode(2, INPUT);
//  cli();
//  EICRA |= (0<<ISC00)|(1<<ISC01);  // EICRA sets interrupt type for INT0...3
//  EICRA |= (0<<ISC10)|(1<<ISC11);
//  EIMSK |= (1<<INT0);
//  EIMSK |= (1<<INT1);
//  sei();
////  cli();
////  //INT6,2,3 set for falling edge
////  EICRB |= (0<<ISC60)|(1<<ISC61); // sets the interrupt type for EICRB (INT6). 
////  // activates the interrupt. 6 for 6, etc
////  EIMSK |= (1<<INT6); 
////  sei();
}

void loop(){    
  if (Serial.available()) {
    processSyncMessage();
  }
 
  //When timeStatus = timeSet displays clock led 13 set to high
  //otherwise wait for sync message, not displaying time led 13 off
  if (timeStatus()!= timeNotSet) {
    digitalClockDisplay();  
//    sleepNow();
//    digitalClockDisplay();
//    Serial.print("outofsleep");
  }
  if (timeStatus() == timeSet) {
    digitalWrite(13, HIGH); // LED on if synced
  } 
  else {
    digitalWrite(13, LOW);  // LED off if needs refresh
  }
  delay(1000);
}

void digitalClockDisplay(){
  // digital clock display of the time
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(day());
  Serial.print(" ");
  Serial.print(month());
  Serial.print(" ");
  Serial.print(year()); 
  Serial.println(); 
}

void printDigits(int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

//gets Time from PC Through serial port, update time if synced time higher 
//than DEFAULT_TIME
void processSyncMessage() {
  unsigned long pctime;
  const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013
  if(Serial.find(TIME_HEADER)) {
     pctime = Serial.parseInt();
     if( pctime >= DEFAULT_TIME) { // check the integer is a valid time (greater than Jan 1 2013)
       setTime(pctime); // Sync Arduino clock to the time received on the serial port
     }
  }
}

//sends request to Sync time and date
time_t requestSync(){
  Serial.write(TIME_REQUEST);  
  return 0; // the time will be sent later in response to serial mesg
}

void sleepNow(){
    /* Now is the time to set the sleep mode. In the Atmega8 datasheet
     * http://www.atmel.com/dyn/resources/prod_documents/doc2486.pdf on page 35
     * there is a list of sleep modes which explains which clocks and 
     * wake up sources are available in which sleep modus.
     *
     * In the avr/sleep.h file, the call names of these sleep modus are to be found:
     *
     * The 5 different modes are:
     *     SLEEP_MODE_IDLE         -the least power savings 
     *     SLEEP_MODE_ADC
     *     SLEEP_MODE_PWR_SAVE
     *     SLEEP_MODE_STANDBY
     *     SLEEP_MODE_PWR_DOWN     -the most power savings
     *
     *  the power reduction management <avr/power.h>  is described in 
     *  http://www.nongnu.org/avr-libc/user-manual/group__avr__power.html
     */  
  
 // EIMSK |= (1<<INT0);
  set_sleep_mode(SLEEP_MODE_IDLE);   // sleep mode is set here
  sleep_enable();          // enables the sleep bit in the mcucr register
                             // so sleep is possible. just a safety pin 
//  power_adc_disable();
//  power_spi_disable();
//  power_timer0_disable();
//  power_timer1_disable();
//  power_timer2_disable();
//  power_twi_disable();
  
  //sleep.cpu();
  sleep_mode();            // here the device is actually put to sleep!!
 
                             // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP
//  sleep_disable();         // first thing after waking from sleep:
                            // disable sleep...

// power_all_enable();
   
}

  ISR(INT2_vect) {
    sleep_disable();
    digitalClockDisplay();   
    power_timer0_disable();
    sleepNow();
  }
//    Serial.print("outofsleep");
//    Serial.print("www ");
//    Serial.print(state);
// //   EIMSK |= (0<<INT0);
//    if (state==true){
//      digitalWrite(13, HIGH);
//      Serial.print("high ");
//      state=false;
//    }
//    else{
//      digitalWrite(13, LOW);
//      Serial.print("low ");
//      state=true;
//    }
  

