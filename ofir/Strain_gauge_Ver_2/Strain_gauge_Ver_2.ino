#include <EEPROM.h>

#include <PinChangeInt.h>
#include <PinChangeIntConfig.h>

#include <SoftwareSerial.h>
#include <Wire.h>
#include <Time.h>  

#include <avr/sleep.h>
#include <avr/power.h>

//Bluetooth UART port 
#define RXBLT 10
#define TXBLT 11

//ADXL345 device address 
#define DEVICE (0x53)  
//num of bytes we are going to read each time ,two bytes for each axis
#define TO_READ (6)       
//tilt roll delta
#define tiltPrePostDelta 25
#define rollPrePostDelta 25
#define tiltVertical 90
#define rollVertical 0
#define MeasNum 10
#define PRE_VAR 3//bra on, tilt,roll
#define POST_VAR 3//bra on, tilt,roll
//pressed button states
#define IDLE_STATE 0 //sleep mode
#define NEW_SESSION 1 //first pressed with no side select if selected go to pre else wait for 5 minutes 
#define PRE_STATE 2 //
#define POST_STATE 3 //after post save and  timer for 10 minutes to create BLT seesion  
//#define END_SESSION 4
#define BLT_SEESION 5// transfer data in EEPROM to Android

//switch  left right indications
#define LEFT_SIDE 10
#define RIGHT_SIDE 11
#define NO_SIDE_SELECT 12

//RTC defined parameters, Time message example T1357041600
#define TIME_HEADER  "T"   // Header tag for serial time sync message
#define TIME_REQUEST  7    // ASCII bell character requests a time sync message 
#define TIME_MSG_LEN 11 // time sync to PC is HEADER followed by Unix time_t

volatile int current_state = IDLE_STATE;
int breast_side = NO_SIDE_SELECT;

//Switches,button  and Buzzer
int buzzer = 5;              //D5 PWM for buzzer, pin31 PC6
int left_side = 8;     //D8  left switch PCINT4 pin 28 PB4
int right_side = 6;  //D6, pin 27 PD7 Right switch No PCINT pin
int INT6_pin = 7;         // D7, pin1 PE6
int SG_Control = 4; //D4  pin 25 PD4
int key =9; //D9  pin 29(PB5)
int sensorPin = A0; // pin36 (PF7) this pin will connect to INA211 output

//timers varialbels, Timer1
boolean timer5MinRun =false;
volatile boolean timer5MinPass=false;
boolean timer3MinRun =false;
volatile boolean timer3MinPass=false;
boolean timer20MinRun =false;
volatile boolean timer20MinPass=false;

//counts timer1 , each 4sec counter++
volatile int 5minCounter=0;
volatile int 3minCounter=0;
volatile int 20minCounter=0;

byte buff[TO_READ] ;    //6 bytes buffer for saving data read from the ADXL345
int regAddress = 0x32;  //first axis-acceleration-data register on the ADXL345

//INA output variabels
int sensorValue = 0;    // variable to store the value coming from the strain gauge sensor
float voltage;               // convert ADC code to voltage representation
int InaThreshold=34; // 50gram threshold  minimum value

int ledPin = 13;        //LED connected to digital pin 13, will be removed
char str[512];          //string buffer to transform data before sending it to the serial port
//volatile  boolean state=false;
//volatile  boolean state1=false;

volatile boolean INT3Flag=false;
volatile boolean INT2Flag=false;

int i=1;
boolean synctime = false;
int timeSyncData = 0;   // for incoming serial data

//adxl345 parameters
float preTilt;//store tilt angle
float preRoll;//strore roll angle


//INT6_press options
volatile boolean  BLE_Button=false;
volatile boolean Countinue=false;

float milkResult[];//hold final result of tilt,roll,sensorMeas per measurement
float preMilkResult[];
float postMilkResult[];

void setup()
{
//InitialiseInterrupt();
  SoftwareSerial BTSerial(RXBLT, TXBLT); // RX | TX
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(9600);  // start serial for output
  BTSerial.begin(9600);  // HC-05 default speed in AT command more
  setSyncProvider( requestSync);  //set function to call when sync required
  analogReference(EXTERNAL);// AREf value for Arduino.
  pinMode(ledPin, OUTPUT); 
  pinMode(SG_Control, OUTPUT); 
  digitalWrite(SG_Control, HIGH); // SG bridge NC to GND
  
  //HC-05 initialize to comm mode 
 pinMode(key, OUTPUT);  // this pin will pull the HC-05 pin 34 (key pin) LOW to switch module to coommand mode
 digitalWrite(key, LOW);    

pinMode(INT6_pin, INPUT);// check if need to configure the interupt pin as input pin

  cli();//stop interrupts
   
  //set timer1 interrupt at 0.25Hz, interupt in 4sec gap.
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 62499;// = (16*10^6) / (0.25*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
//  TIMSK1 |= (1 << OCIE1A);
   
//INT6,2,3 set for falling edge
 EICRB |= (0<<ISC60)|(1<<ISC61); // sets the interrupt type for EICRB (INT6). 
 EICRA |= (0<<ISC30)|(1<<ISC31);  // EICRA sets interrupt type for INT0...3
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
 
 sei();//allow interrupts
 
 //Turning ON the ADXL345
  writeTo(DEVICE, 0x2D, 0);      
  writeTo(DEVICE, 0x2D, 16);
  writeTo(DEVICE, 0x2D, 8);

//enters to sleep mode need to check idle Vs. standby ,INT6 exit from sleep
enterSleep();
}

void loop(){
  
  if (current_state==BLT_SEESION){
  }
  /*
  start 5min timer
  */
  if (current_state==NEW_SESSION){
    if (timer5MinRun==false){
        // enable timer compare interrupt, start counting 5min
         cli();
         TIMSK1 |= (1 << OCIE1A);
         sei();
         timer5MinRun=true;
         timer5MinPass=false;
    }
    /* 
    5 minute timer until selection of breast side go to PRE_STATE
    else go to idle
    */
     newSession();
  }
  
  /*
  recive from zeroTracking()  boolean Array  {braOn,preDeltaTilt,preDeltaRoll} 
  if average parameters stable, verify meas. side and start measurement
  if measurement percentage not over threshold for more than 10 tries
  return to idle state.
  if pre meas. test not good send error message zero tracking error.
  if measurement not good send measurement stability error.
  */
  if (current_state==PRE_STATE){
    int i;
    int trueCount=0;
    boolean measStability;
    boolean preSetup[PRE_VAR];
   //float  measReasult[3];
    //start timer1 for 3 min to check if valid standing and bra on conditions occour
     if (timer3MinRun==false){
        // enable timer1 compare interrupt, start counting 3min
         cli();
         TIMSK1 |= (1 << OCIE1A);
         sei();
         timer3MinRun=true;
         timer3MinPass=false;
    }
    if(timer3MinPass==false){
        preSetup=zeroTracking(); //returns boolean[ PRE_VAR] array {braOn,tilt,roll}
        for ( i=0;i<PRE_VAR;i++){  
          if (preSetup[i]==true){
             trueCount++;
          }
        }
        if (trueCount==PRE_VAR){
         cli();
         TIMSK1 |= (0 << OCIE1A);//timer1 disable, gyroZeroTracking OK
         sei();
         timer3MinRun=false;
         timer3MinPass=false;
         breast_side=checkMeasSide();
         measStability=milkMesurement(); TBD make measReasult golbal parameter and return bollean
         if(measStability==false){
            current_state=IDLE_STATE;
            errMeasStability();
         }
         if(measStability==true){
           preMilkResult= milkResult;
           current_state=NURSERING_STATE;
         }
          
          /*
          start 20min counter until moving to post state
          if 20min pass go to IDLE
          */
        }
    }
    if(timer3MinPass==true){
      cli();
      TIMSK1 |= (0 << OCIE1A);//timer1 disable
      sei();
      gyroZeroTrackingError(preSetup);
      current_state=IDLE;
    }
    }
  
  
   
 //   sensorValue = analogRead(sensorPin);
 //check threshold  
    //check tilt roll stability
    //check selected side
    //start measurment
    //start counter for 20min  
 
  
   /*
   counts 20min until int6 button pressed,
   if 20min pass return to idle state if not continue to post state.
   */
   if(current_state==NURSERING_STATE){ 
      if(timer20MinRun==false){
        cli();
        TIMSK1 |= (1 << OCIE1A);
        sei(); 
        timer20MinRun=true;
        timer20MinPass=false;
      }
     if(timer20MinPass==true){
       current_state=IDLE_STATE;//TBD:check if needed to restart preMilkResult postMilkResult arrays
     }
   }
    /*
    check zero tracking  
    check selected side if changed go to 3min wait time if not returned 
    to prestate side go to idle.
    take measurement verify results
    if pre weight> post weight save
    else try take measurement postRepatbiilty times.
    change current_state to DATA_DOWNLOAD.
    */
   if(current_state==POST_STATE){

    int i;
    int trueCount=0;
    int postRepatbiilty=3;
    int currentRepatbilty=0;
    boolean measStability;
    boolean postSetup[POST_VAR];
   //float  measReasult[3];
    //start timer1 for 3 min to check if valid standing and bra on conditions occour
    if (timer3MinRun==false){
      // enable timer1 compare interrupt, start counting 3min
      cli();
      TIMSK1 |= (1 << OCIE1A);
      sei();
      timer3MinRun=true;
      timer3MinPass=false;
    }
    if(timer3MinPass==false){
      postSetup=zeroTracking(); //returns boolean[ POST_VAR] array {braOn,tilt,roll}
        for(i=0;i<POST_VAR;i++){  
          if (postSetup[i]==true){
            trueCount++;
          }
        }
        //zero tracking pass OK
        if(trueCount==POST_VAR){
          cli();
          TIMSK1 |= (0 << OCIE1A);//timer1 disable, gyroZeroTracking OK
          sei();
          timer3MinRun=false;
          timer3MinPass=false;
          checkMeasPostSide(breast_side); //TBD fast led flash for 3min if 3min pass go to idle
          for(i=0;i<postRepatbailty;i++){
            measStability=milkMesurement();
            if(measStability==false){
              current_state=IDLE_STATE;
              errMeasStability();
            }
            else{
             postMilkResult= milkResult;
             if(preMilkResult[PRE_VAR-1]>postMilkResult[POST_VAR-1]){
               saveSession();
               current_state=DATA_DOWNLOAD;
               break;++;
             }
            }
          }
        }
    }
    else{ //if(timer3MinPass==true){
      cli();
      TIMSK1 |= (0 << OCIE1A);//timer1 disable
      sei();
      gyroZeroTrackingError(preSetup);
      current_state=IDLE;
    }
   }   
  }
  /*
  check if BLT session in pair mode if yes download seesion
  and go to idle state TBD
  */
  if (current_state==DATA_DOWNLOAD){
    
  } 
   /*
   Verify threshold value if Ok start measurement and make PWM BUZZER sound
   if threshold not OK red led blinks if more than 5 minutes go to sleep
   after meas.  check if valid save in EEPROM
  */
{
 
 void saveSession(){
 
 }
  
  
  
  /*
  create measurements milkMeasurement[tilt,roll,meas,tilt,roll,meas,......] array length
  as milkMeasLength interger.
  for each measurement go through tilt and roll values if values out of range dont use meas values.
  average every 10 legal measurements ( tilt and roll OK).
  create array goodMeas[], if length<minLength repeat measurement X measRepatability integer.
  create array  milkMeasAvg[tiltAvg,rollAvg,MeasAvg......] milkMeasAvg/10 length.
  averge the milkMeasAvg array to create  : 
  milkResult[tiltAvg,rollAvg,measAvg] milkResult length 3.
  return array.
  Ina/sensor values return as int between 0 to 1023
  */
  float[] milkMesurement(int measRepatability,int tiltRef,int tiltDelta,int rollRef,int rollDelta){
    int measNum=1000;
    int i,j,y;
    float milkMeasurement[];
    float goodMeas[];
    float milkMeas10Avg[];
    //Golbal parm float milkResult[];
//    float tiltRoll[2*measNum];
    float tiltRoll[2];
    TBD  add missing parameters.
 
    for(i=0; i<measRepatability;i++){
      int k=0;
      for(j=0; j<measNum;j++){//reads measNum times gyro values, if offset Ok create weight measurement.
        tiltRoll=gyroRead(1);
        tiltRollValid=checkTiltRollOffset(tiltRoll[0],tiltRoll[1]);// TBD write checkTiltRollOffset
        if(tiltRollValid==true){
          milkMeasurement[k]=tiltRoll[0]
          milkMeasurement[k+1]=tiltRoll[1]
          milkMeasurement[k+2]=analogRead(sensorPin);
          k+=3;
        }
      }
      if(k>0.9*3*measNum){//90% of measurements OK,milkMeasurement array length 90% alteast from max length
          int actualMeas=(k/3);
          int actualMeas10Avg=actualMeas/10;
          float tilt[actualMeas];
          float roll[actualMeas];
          float sensorMeas[actualMeas];
          //create 3 arrays of tilt,roll,weight measurement
          int z;
          for(z=0;(z/3)<actualMeas;z=z+3){
            tilt[z/3]=milkMeasurement[z];
            roll[z/3]=milkMeasurement[z+1];
            sensorMeas[z/3]=milkMeasurement[z+2];
          }
          for(z=0;z<actualMeas;z+=10){
            tiltSum=0;
            rollSum=0;
            measSum=0;
            for(y=0;y<10;y++){
              tiltSum+=tilt[z+y];
              rollSum+=roll[z+y];
              measSum+=sensorMeas[z+y];
            }
            tilt10Avg=tiltSum/10;
            roll10Avg=rollSum/10;
            meas10Avg=measSum/10;
            milkMeas10Avg[(z/10)*3]=tilt10Avg;
            milkMeas10Avg[(z/10)*3+1]=roll10Avg;
            milkMeas10Avg[(z/10)*3+2]=meas10Avg; //milkMeas10Avg[tilt[0..9]Avg,roll[0..9]avg,meas[0..9]avg],tilt[10..19]Avg,roll[10..19]avg,meas[10..19]avg] ...X actualMeas10Avg counts
          }
          for(z=0;(z/3)<actualMeas10Avg;z+=3){
            tiltFinalAvg=milkMeas10Avg[z];
            rollFinalAvg=milkMeas10Avg[z+1];
            measFinalAvg=milkMeas10Avg[z+2];
          }
          milkResult[0]=tiltFinalAvg;
          milkResult[1]=rollFinalAvg;
          milkResult[2]=measFinalAvg;
      }
      return true;
    }
    return false;
  }
  /*
  check what meas side has been selected and indicate leds.
  TBD led indications
  */
int  checkMeasSide(){
  int BreastSide;
  if(Digitalread(left_side)==LOW && Digitalread(right_side)==LOW){
     BreastSide=NO_SIDE_SELECT;
  {
  if(Digitalread(left_side)=HIGH){
     BreastSide=LEFT_SIDE;
  }
  else{
     BreastSide=RIGHT_SIDE;
  }
  return BreastSide;
}

void gyroZeroTrackingError( boolean preSetup[]){
  int i;
  for(i=0;i<PRE_VAR;i++){
    switch (i) {
      case 1:
        if (preSetup[0]==false){
          Serial.print("bra threshold low:");
          Serial.print(sensorValue,DEC);
      }
      case 2:
         if (preSetup[1]==false){
           Serial.print("Pre tilt vertical offset too high");
           Serial.print(preTilt,DEC);
        }
        case 2:
         if (preSetup[2]==false){
           Serial.print("Pre roll vertical offset too high");
            Serial.print(preRoll,DEC);
        }
}
 
void newSession(){ 
      if (Digitalread(left_side)==LOW && Digitalread(right_side)==LOW ){
        if(timer5MinPass == true){
              current_state=IDLE;
              timer5MinRun=false;
    //          timer5MinPass =false;// turned off only when timer triggired on
              cli();
               //  5min timer over, disable timer go to idle mode
              TIMSK1 |= (0 << OCIE1A);
              sei();
              enterSleep();//enter sleep mode
        }
        else{
              breast_side = NO_SIDE_SELECT;
        }
  
      if (Digitalread(left_side)==HIGH & Digitalread(right_side)==LOW){
        breast_side = LEFT_SIDE;
      }
       if (Digitalread(left_side)==LOW & Digitalread(right_side)==HIGH){
        breast_side = RIGHT_SIDE;
      }
      //side has been choosen
      if (breast_side !=NO_SIDE_SELECT){
        cli();
        // breast side has been choosed, diable 5min timer
         TIMSK1 |= (0 << OCIE1A);
         sei()
        //indicate led side turn on
        timer5MinRun=false;
        timer5MinPass =false;
        current_state=PRE_STATE;
      }
}

ISR(INT6_vect){
  if (current_state==IDLE_STATE){
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
/* 
checks if bra in on body and if woman standing is in 
required delta from vertical position, return boolean array values
{braOn,preDeltaTilt,preDeltaRoll}
*/
boolean[] zeroTracking(){
    boolean braOn ;
    boolean VerticalDelta;
    float TiltRollValues[2]; //{tilt,roll}
    boolean preMeasSetup[PRE_VAR];
    sensorValue = analogRead(sensorPin);
    if (sensorValue>braThreshold){
      braOn =true;
    else {
      braOn=false;
    }    
    preMeasSetup[0]=braOn;
    TiltRollValues=gyroRead(2); //return float array of [tilt,roll]
    if (rollVertical-preTiltDelta<TiltRollValues[0)<rollVertical+preTiltDelta){
      preMeasSetup[1]=true;
    }
    else{
      preMeasSetup[1]=false;
    }
    
    if(rollVertical- preRollDelta <TiltRollValues[1]<rollVertical+preRollDelta){
      preMeasSetup[2]=true;
   }
   else(
     preMeasSetup[2]=false;
   }
   return preMeasSetup;//boolean array [braOn,tilt,roll]
}
/*
  reads tilt,roll values from gyro for MeasNum times. if all values
  in offset range return boolean array {tilt,rool} of true values else
  one of the array values will be false.
*/
boolean[] gyroZeroTrackingRead(int tiltRef,int tiltDelta,int rollRef,int rollDelta){
//  boolean preSetup;
  float tiltRoll[2*MeasNum];;
//  float tiltSum;
//  float rollSum;
  int tiltCounter=0;
  int rollCounter=0;
  boolean[2] zeroTrackingTiltRoll;
  tiltRoll=gyroread(2*MeasNum);
  int i;
  for(i=0;i<2*MeasNum;i=i+2){
    if (tiltRef - tiltDelta < tiltRoll[i] < tiltRef + tiltDelta){
         tiltCounter++;
         //  preMeasSetup[1]=true;
    }
    if (rollRef - rollDelta < tiltRollroll[i+1] < rollRef + rollDelta){
       rollCounter++;
    //  preMeasSetup[1]=true;
    }
//    return sensorValue;  
 //    voltage= sensorValue * (3.3/ 1023.0);
  }
    if(tiltCounter==MeasNum ){
       zeroTrackingTiltRoll[0]=true;
    }
    else{
      zeroTrackingTiltRoll[0]=false;
    }
    if(rollCounter==MeasNum){
       zeroTrackingTiltRoll[1]=true;
    }
    else{
       zeroTrackingTiltRoll[1]=false;
    }
//    for (i = 0; i < MeasNum; i = i + 1){
//      tiltSum+=tilt[i];
//      rollSum+=roll[i];
//    }
//    TiltRoll[0]=tiltSum/MeasNum;
//    TiltRoll[1]=rollSum/MeasNum;
    return zeroTrackingTiltRoll;
}
  /*
  reads gyro values [tilt,roll * readNum times]
  returns tiltRoll[2*readNum] float values of the reading.
  */
  float[] gyroRead(int readNum){
    int i;
    int x, y, z;//variables to store ADXL345 reading
    float xg,yg,zg;//These variables will be used to hold the x,y and z axis accelerometer values in G's values.
    float mag;//store the magnitued of xg,yg,zg
    float tiltRoll[2*readNum];
    //float tilt[MeasNum];
    for (i = 0; i < readNum; i=i+2){
      readFrom(DEVICE, regAddress, TO_READ, buff);  
       /*
       each axis reading comes in 10 bit resolution, ie 2 bytes.  Least Significat Byte first!!
       thus we are converting both bytes in to one int
      */   
      x = (((int)buff[1]) << 8) | buff[0];   
      y = (((int)buff[3])<< 8) | buff[2];
      z = (((int)buff[5]) << 8) | buff[4];
   /*
      Convert the accelerometer value to G's. 
      With 10 bits measuring over a +/-4g range we can find how to convert by using the equation:
      Gs = Measurement Value * (G-range/(2^10)) or Gs = Measurement Value * (8/1024)
   */
      xg = x * 0.0078;
      yg = y * 0.0078;
      zg = z * 0.0078;
  
      mag=sqrt(pow(xg,2)+pow(yg,2)+pow(zg,2));
      tilt=acos(zg/mag)*(180/PI);
      roll = atan2(yg,sqrt(pow(xg,2)+pow(zg,2)));
      roll=roll[i]*(180/PI);
      tiltRoll[i]=tilt;
      tiltRoll[i+1]=roll;
    //calculating the roll angle , rotaion angle around Vector X
    }
    return tiltRoll;
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
  

  
 // Serial.println("roll" ); 
 // Serial.println(roll, DEC );

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
  if ( INT3Flag==true){
        analogWrite(PWMpin, 125);
  }
  else{
         analogWrite(PWMpin, 0);
  }
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
  Wire.requestFrom(device, num);      //request 6 bytes from device
  
  int i = 0;
  while(Wire.available())    //device may send less than requested (abnormal)
  { 
    buff[i] = Wire.read(); // receive a byte
    i++;
  }
  Wire.endTransmission(); //end transmission
}

//
void updateState(){
   switch (current_state) {
    case IDLE_STATE:
      current_state=NEW_SESSION;
      break;
    case NURSERING_STATE:
      current_state=POST_MEASUREMENT;
      cli();
      TIMSK1 |= (0 << OCIE1A);//timer1 disable
      sei();
      prePostTimeError();
      current_state=IDLE;
      break;
     case POST_STATE:
      current_state=POST_MEASUREMENT
      break;
    default: 
      // if nothing else matches, do the default
      // default is optional
  }
  
  /*timer1 interrupt 0.25Hz, indication if 5min past since session mode
  if yes go to idle
  */
  ISR(TIMER1_COMPA_vect){
   5minCounter++;
   3minCounter++;
   20minCounter++;
   if(5minCounter==75){
     timer5MinPass=true;//5minpass
     5minCounter=0;
   }
   if(3minCounter==45){
     timer3MinPass=true;//3minpass
     3minCounter=0;
   }
    if(20minCounter==300){
     timer20MinPass=true;//20minpass
     20minCounter=0;
   }
  }
//  if (sideNoChange()){
//    SWITCHcase update by 1
//  }
//  else{
//      if(breast_side==LEFT_SIDE){
//        breast_side=RIGHT_SIDE;
//      {
//        else{
//         breast_side=LEFT_SIDE;
//        }
//  }
//}
  
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
