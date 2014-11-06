#include <avr/sleep.h>
#include <Time.h> 
#include <avr/power.h>  

#define TIME_HEADER  "T"   // Header tag for serial time sync message
#define TIME_REQUEST  7    // ASCII bell character requests a time sync message 

boolean syncTime=false;
//T1357041600
void setup(){
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  pinMode(2, INPUT);
  digitalWrite(13,HIGH);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
//  set_sleep_mode(SLEEP_MODE_IDLE);
  setSyncProvider( requestSync);  //set function to call when sync required
  Serial.println("Waiting for sync message");
//  noInterrupts();
  while(syncTime==false){
    if (Serial.available()) {
      processSyncMessage(); 
      digitalClockDisplay();
      syncTime=true;
    }
  }
//  interrupts();
//  cli();
//  EICRA |= (0<<ISC00)|(0<<ISC01);  // EICRA sets interrupt type for INT0...3
//  EICRA |= (0<<ISC10)|(0<<ISC11);
//  EIMSK |= (1<<INT0);
//  EIMSK |= (1<<INT1);
//  sei();
}

void loop(){
  Serial.println("back to loop");
  delay(5000);
  sleepSetup();  
}

void sleepSetup(){
 // if (timeStatus()!= timeNotSet) {
 //   digitalClockDisplay();
 // }
  //EIMSK |= (1<<INT0);
 // noInterrupts();
  Serial.println("going to sleep");
    delay(1000);

  attachInterrupt(0,pinInterrupt,LOW);
  sleep_enable();
    sleep_disable();

  digitalWrite(13,LOW);
  //interrupts();
  sleep_cpu();
  //sleep_disable();
  digitalClockDisplay();
  Serial.println("just woke up");
  digitalWrite(13,HIGH);
}

void pinInterrupt(){
//  noInterrupts();
  //detachInterrupt(0);
  //EIMSK |= (0<<INT0);
  Serial.println("INT2");
//  interrupts();
  sleep_disable();
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


