#include <PinChangeInt.h>
#include <PinChangeIntConfig.h>

#include <SoftwareSerial.h>
#include <Wire.h>
#include <Time.h>  

#include <avr/sleep.h>
#include <avr/power.h>

//Bluetooth UART port 
SoftwareSerial BTSerial(10, 11); // RX | TX

//ADXL345 device address 
//num of bytes we are going to read each time (two bytes for each axis)
#define DEVICE (0x53)  
#define TO_READ (6)       

//Switches
#define Sw_2 31               //         D5 PWM for buzzer
#define Sw_Left 28     //       left switch PCINT4
#define Sw_Right 27  //      PD7 Right switch (No PCINT pin)
int INT6_pin=7;//          int 6 pin

//RTC defined parameters, Time message example T1357041600
#define TIME_HEADER  "T"   // Header tag for serial time sync message
#define TIME_REQUEST  7    // ASCII bell character requests a time sync message 
#define TIME_MSG_LEN 11 // time sync to PC is HEADER followed by Unix time_t

//pressed button states
#define IDLE_STATE 0
#define NEW_SEESION 1
#define SIDE_SELECT 2
#define PRE_STATE 3
#define POST_STATE 4
#define BLT_SEESION 5
#define LEFT_SIDE 6
#define RIGHT_SIDE 7
#define NO_SIDE_SELECT 8

int current_state = IDLE_STATE;
int step_count = 1;
int current_breast = NO_SIDE_SELECT

byte buff[TO_READ] ;    //6 bytes buffer for saving data read from the ADXL345
int regAddress = 0x32;  //first axis-acceleration-data register on the ADXL345

//port for PWM also defined in defined section
int PWMpin=5; //digital PIN 5 used for PWM output

//INA output variabels
int sensorPin = A0;     // this pin will connect to INA211 output
int sensorValue = 0;    // variable to store the value coming from the strain gauge sensor
float voltage;          // convert ADC code to voltage representation

int ledPin = 13;        //LED connected to digital pin 13, will be removed
char str[512];          //string buffer to transform data before sending it to the serial port
volatile  boolean state=false;
volatile  boolean state1=false;

volatile boolean INT3Flag=false;
volatile boolean INT2Flag=false;
int i=1;
boolean synctime = false;
int timeSyncData = 0;   // for incoming serial data

//adxl345 parameters
int x, y, z;//variables to store ADXL345 reading
float xg,yg,zg;//These variables will be used to hold the x,y and z axis accelerometer values in G's values.
float mag;//store the magnitued of xg,yg,zg
float tilt;//store tilt angle
float roll;//strore roll angle

//INT6_press options
volatile boolean  BLE_Button=false;
volatile boolean Countinue=false;

//measurement side indication
volatile int left_side =0;
volatile int right_side =1;



void setup()
{
//InitialiseInterrupt();
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(9600);  // start serial for output
  BTSerial.begin(9600);  // HC-05 default speed in AT command more
  setSyncProvider( requestSync);  //set function to call when sync required
  pinMode(ledPin, OUTPUT); 
//PCintPort::attachInterrupt(8, change,FALLING); // attach a PinChange Interrupt to our pin on the rising edge
   
//Turning ON the ADXL345
  writeTo(DEVICE, 0x2D, 0);      
  writeTo(DEVICE, 0x2D, 16);
  writeTo(DEVICE, 0x2D, 8);
 
//HC-05 initialize to comm mode 
 pinMode(9, OUTPUT);  // this pin will pull the HC-05 pin 34 (key pin) LOW to switch module to coommand mode
 digitalWrite(9, LOW);    
 analogReference(EXTERNAL);// AREf value for Arduino.

pinMode(INT6_pin, INPUT);// check if need to configure the interupt pin as input pin
//INT6,2,3 set for falling edge
 EICRB |= (0<<ISC60)|(1<<ISC61); // sets the interrupt type for EICRB (INT6). 
 EICRA |= (0<<ISC30)|(1<<ISC31);                                 // EICRA sets interrupt type for INT0...3
 EICRA |= (0<<ISC20)|(1<<ISC21);
 /*
 ISCn0  ISCn1   Where n is the interrupt. 0 for 0, etc
  0      0   Triggers on low level
  1      0   Triggers on edge
  0      1   Triggers on falling edge
  1      1   Triggers on rising edge
 */
 
// activates the interrupt. 6 for 6, etc
 EIMSK |= (1<<INT6); 
 EIMSK |= (1<<INT2);
 EIMSK |= (1<<INT3);

//set SG control leg to HIGH level TBD, sensor bridge open

//enters to sleep mode need to check idle Vs. standby
//press on INT6 exit from sleep
enterSleep();

}

void loop(){
  //micro out of sleep in measurement mode
 if (continue){
  //measurement status led ON
  //choose left or right side, each time interrput side  changed 
  
  
  
   //start pre or post measurement again
   //indicate led side
   
   //INT6 press once make pre measurement, indicate led in flash mode
   
   //Verify threshold value if Ok start measurement and make PWM BUZZER sound
   //if threshold not OK red led blinks if more than 5 minutes go to sleep
   //after meas.  check if valid save in EEPROM
   

   
   //
   
   
   
   
{
  //micro out of sleep in Bluetooth mode  
 if (  BLE_Button){ 
   //led Bluetooth ON
 }
  
 
  
  //wait until sync message arrived from BLT serial port  
 if ( i==1){
     delay(5000);
     BTSerial.println("set time");
     i++;
}
  while(synctime != true){ 
    if (BTSerial.available() >= TIME_MSG_LEN ){
      processSyncMessage();
      synctime=true;
    }
  }
  
 //read the acceleration data from the ADXL345
  readFrom(DEVICE, regAddress, TO_READ, buff);
  
   //each axis reading comes in 10 bit resolution, ie 2 bytes.  Least Significat Byte first!!
   //thus we are converting both bytes in to one int
  x = (((int)buff[1]) << 8) | buff[0];   
  y = (((int)buff[3])<< 8) | buff[2];
  z = (((int)buff[5]) << 8) | buff[4];
  
   //Convert the accelerometer value to G's. 
  //With 10 bits measuring over a +/-4g range we can find how to convert by using the equation:
  // Gs = Measurement Value * (G-range/(2^10)) or Gs = Measurement Value * (8/1024)
  xg = x * 0.0078;
  yg = y * 0.0078;
  zg = z * 0.0078;
  
  mag=sqrt(pow(xg,2)+pow(yg,2)+pow(zg,2));
  tilt=acos(zg/mag)*(180/PI);
  //  Serial.println("tilt" ); 
  // Serial.println(tilt, DEC ); 
  //calculating the roll angle , rotaion angle around Vector X
  roll = atan2(yg,sqrt(pow(xg,2)+pow(zg,2)));
  roll=roll*(180/PI);
  
 // Serial.println("roll" ); 
 // Serial.println(roll, DEC );
  sensorValue = analogRead(sensorPin);
  voltage= sensorValue * (3.3/ 1023.0);
// if ( INT2Flag==true) {

  //display time after sync from BLT
  digitalClockDisplay(); 

  // display Voltage, tilt and roll angels
  BTSerial.write("voltage");  
  BTSerial.write(10);
  BTSerial.println(voltage,DEC);
  
  BTSerial.write("tilt");  
  BTSerial.write(10);
  BTSerial.println(tilt,DEC);
   
  BTSerial.write("roll");  
  BTSerial.write(10);
  BTSerial.println(roll,DEC);
  
//}
  //we send the x y z values as a string to the serial port
 // sprintf(str, "%d %d %d", x, y, z);  
  //Serial.print(str);
 //Serial.println(" ");
//  Serial.write(10);
  if ( INT3Flag==true)
        analogWrite(PWMpin, 125);
  else
         analogWrite(PWMpin, 0);
 // Serial.print(state1);
  //It appears that delay is needed in order not to clog the port
  delay(2500);
  
  //enter sleep mode, 
  //exits from sleep when INT6_pin pressed in first time.
  enterSleep();
}
  

//---------------- Functions
//Writes val to address register on device
void writeTo(int device, byte address, byte val) {
 Wire.beginTransmission(device); //start transmission to device 
 Wire.write(address);        // send register address
 Wire.write(val);        // send value to write
 Wire.endTransmission(); //end transmission
}

//reads num bytes starting from address register on device in to buff array
void readFrom(int device, byte address, int num, byte buff[]) {
Wire.beginTransmission(device); //start transmission to device 
Wire.write(address);        //sends address to read from
Wire.endTransmission(); //end transmission
  
  Wire.beginTransmission(device); //start transmission to device
  Wire.requestFrom(device, num);    // request 6 bytes from device
  
  int i = 0;
  while(Wire.available())    //device may send less than requested (abnormal)
  { 
    buff[i] = Wire.read(); // receive a byte
    i++;
  }
  Wire.endTransmission(); //end transmission
}

//both switch input
ISR(INT6_vect){
  // interrupt code goes here
  if (current_state==IDLE){
    delayMicroseconds(3000000);//3 sec delay
    if (digitalRead(INT6_pin)==LOW){
      current_state=BLT_SEESION;
    }
    else{
      //Countinue=true;
      updateState();
    }
 // state=!state; // toggle running variable
 //digitalWrite(ledPin, state);
}

//
void updateState(){
  if (sideNoChange()){
    SWITCHcase update by 1
  }
  else{
      if(current_breast==LEFT_SIDE){
        current_breast=RIGHT_SIDE;
      {
        else{
         current_breast=LEFT_SIDE;
        }
  }
}
  
// void change() {
//  // interrupt code goes here
//  state1=!state1; // toggle running variable
//  digitalWrite(ledPin, HIGH);
//  }
  
  //RX (INT2) falling edge ,controll BLE transmission
  ISR(INT2_vect) {
    INT2Flag =! INT2Flag;
  // interrupt code goes here
//  state=!state; // toggle running variable
//  digitalWrite(ledPin, state);
  }
  //TX (INT3) falling edge,controll PWM transmission
  ISR(INT3_vect) {
     INT3Flag =! INT3Flag;
  }

//void InitialiseInterrupt(){
//  cli();		// switch interrupts off while messing with their settings  
//  PCICR =0x01;          // Enable PCINT1 interrupt
//  PCMSK0 = 0x10;
//  sei();
//}
// ISR(PCINTO_vect) {    // Interrupt service routine. Every single PCINT8..14 (=ADC0..5) change  // will generate an interrupt: but this will always be the same interrupt routine
// state1 =!state1; // toggle running variable
// digitalWrite(6, HIGH);
//}


//gets Time from PC Through BLT serial port, update time if synced time higher 
//than DEFAULT_TIME
void processSyncMessage() {
  unsigned long pctime;
  const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013

  if(BTSerial.find(TIME_HEADER)) {
     pctime = BTSerial.parseInt();
     if( pctime >= DEFAULT_TIME) { // check the integer is a valid time (greater than Jan 1 2013)
       setTime(pctime); // Sync Arduino clock to the time received on the serial port
     }
  }
}

//sends request to Sync time and date
time_t requestSync()
{
  BTSerial.write(TIME_REQUEST);  
  return 0; // the time will be sent later in response to serial mesg
}

void digitalClockDisplay(){
  // digital clock display of the time
  BTSerial.print(hour());
  printDigits(minute());
  printDigits(second());
  BTSerial.print(" ");
  BTSerial.print(day());
  BTSerial.print(" ");
  BTSerial.print(month());
  BTSerial.print(" ");
  BTSerial.print(year()); 
  BTSerial.println(); 
}

void printDigits(int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  BTSerial.print(":");
  if(digits < 10)
    BTSerial.print('0');
  BTSerial.print(digits);
}

//enters the arduino into sleep mode.
//wakes up for
void enterSleep(void)
{  
  /* Setup INT6 as an interrupt and attach handler. */
  //attachInterrupt(0, pin2Interrupt, LOW);
  //delay(100);
  
  set_sleep_mode(SLEEP_MODE_STANDBY); 
  sleep_enable();
  sleep_mode();
  
  /* The program will continue from here. */
  /* First thing to do is disable sleep. */
  sleep_disable(); 
}
