#include <SPI.h>

#define MISO 12
#define SCLK 13
#define FILTER 11
#define PDRST 10
#define GAIN 9

byte byte1; byte byte2; byte byte3; byte byte4;
// declare 3 bytes = 24 bits

void setup()
{
  Serial.begin(9600);
//    pinMode(FILTER, OUTPUT); 
//    pinMode(PDRST, OUTPUT); 
//    pinMode(GAIN, OUTPUT); 
//    pinMode(MISO, INPUT);
  // corresponding to SCK pin and DRDY/DOUT pin on ADC
  
  
  // initialize SPI (with default settings, including...
  // CPOL = 0: so that SCLK is normally LOW
  // CPHA = 0: data sampled on rising edge (LOW to HIGH)
  // perhaps try changing CPHA ??
  SPI.begin();

  //initial parameters  
  digitalWrite(FILTER, LOW);//16.7HZ
  digitalWrite(GAIN, LOW);//128
  digitalWrite(PDRST, HIGH);//Active
}

void loop()
{
    int val= digitalRead(MISO);
    Serial.println(val);
    delay(1000);
    read_adc();
//    Serial.println(byte1);
//    Serial.println(byte2);
//    Serial.println(byte3);
//    Serial.println(byte3);
//    Serial.println(byte4);
  
//  if(byte3 & 0x80){
//        Serial.println("try read conversion");
//     read_adc();
//  }
  // "sort of" an interrupt to go to read_adc routine;
  // can use hardware interrupt in future but now just poll
}

//void reset_adc()
//// to reset ADC, we need SCLK HIGH for min of 4 CONVCYCLES
//// so here, hold SCLK HIGH for 5 CONVCYCLEs = 1440 usec
//{
//  digitalWrite(SCLKPIN, HIGH);
//  delayMicroseconds(1440);
//}

void read_adc()
{
//  drdy_wait();
  // go to drdy_wait routine, where we wait for
  // DRDY phase to pass, and thus for DOUT phase to begin

  byte1 = SPI.transfer(0x00);
  byte2 = SPI.transfer(0x00);
  byte3 = SPI.transfer(0x00);
  byte4 = SPI.transfer(0x00);
  // read in adc data (sending out don't care bytes)
  // and store read data into three bytes */

//  Serial.println(byte1, DEC);
//  Serial.println(byte2, DEC);
//  Serial.println(byte3, DEC);
//  Serial.println();
  // print out data;
  // will these instructions eat into time significantly?
  // possible improvement: store all data from multiple cycles
  // into array, and print out only later at end.
}

void drdy_wait()
// wait for DRDY to pass and to reach start-point of DOUT
{
  delayMicroseconds(30);
  // to be safe, 30 usec, instead of 27 usec, which is
  // the expected period of DRDY phase
}
