#include <SPI.h>


#define PDRST 9
#define GAIN 10
#define FILTER 11
#define MISOPIN 12
#define SCLKPIN 13

byte byte1; byte byte2; byte byte3;byte byte4;
// declare 3 bytes = 32 bits

void setup()
{
  Serial.begin(9600);
  
  // corresponding to SCK pin and DRDY/DOUT pin on ADC
  pinMode(SCLKPIN, OUTPUT); 
  pinMode(PDRST, OUTPUT); 
  pinMode(GAIN, OUTPUT); 
  pinMode(FILTER, OUTPUT); 
  pinMode(MISOPIN, INPUT);

  // initialize SPI (with default settings, including...
  // CPOL = 0: so that SCLK is normally LOW
  // CPHA = 0: data sampled on rising edge (LOW to HIGH)
  // perhaps try changing CPHA ??
  SPI.begin();
 SPI.setBitOrder(MSBFIRST);
 
   // put ADC on reset 
  reset_adc();

  digitalWrite(GAIN, LOW);
  digitalWrite(FILTER, LOW);
  // go to drdy_wait routine, where we wait for
  // DRDY phase to pass, and thus for DOUT phase to begin
  drdy_wait();
  
  // release ADC from reset; now we're at a known point
  // in the timing diagram, and just have to wait for
  // the beginning of a conversion cycle
}

void loop()
{
    // "sort of" an interrupt to go to read_adc routine;
  // can use hardware interrupt in future but now just poll
  if (digitalRead(MISOPIN) == LOW) {
    read_adc();
  }
}

// to reset ADC, we need PDRST LOW for at least 100ns
void reset_adc()
{
  digitalWrite(PDRST, LOW);
  delayMicroseconds(1000); //delay of 1ms.
  digitalWrite(PDRST, HIGH);
}

void read_adc()
{

  // read in adc data (sending out don't care bytes)
  // and store read data into four bytes */
  byte1 = SPI.transfer(0x00);
  byte2 = SPI.transfer(0x00);
  byte3 = SPI.transfer(0x00);
  byte4 = SPI.transfer(0x00);

  // possible improvement: store all data from multiple cycles
  // into array, and print out only later at end.
  Serial.println(byte1, DEC);
  Serial.println(byte2, DEC);
  Serial.println(byte3, DEC);
   Serial.println(byte4, DEC);
  if (byte4 == 0x01 ){
      Serial.println( "serial transfer from the ADC was performed correctly");
  }
}

// wait for DRDY to pass and to reach start-point of DOUT
void drdy_wait()
{
   delayMicroseconds(1300);
}
