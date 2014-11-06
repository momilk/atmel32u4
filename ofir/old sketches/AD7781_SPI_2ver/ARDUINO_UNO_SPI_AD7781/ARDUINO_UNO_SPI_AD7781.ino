#include <SPI.h>

#define PDRST 8
#define GAIN 10
#define FILTER 11
#define MISOPIN 12
#define SCLKPIN 13

byte byte1 ,byte2,byte3,byte4;
byte middle_byte,left_byte;
//word first2Byte;
long code_new=0,code_old=0,code_diff=0;
double weight;
//long dudi;
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
//  SPI.setBitOrder(MSBFIRST);
 
   // put ADC on reset 
  reset_adc();

  digitalWrite(GAIN, LOW);
  digitalWrite(FILTER, LOW);
  digitalWrite(PDRST, HIGH);
  
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
    delay(3000);
//    delayMicroseconds(200);
    read_adc();
  }
}

// to reset ADC, we need PDRST LOW for at least 100ns
void reset_adc()
{
  digitalWrite(PDRST, LOW);
  delay(1); //delay of 1ms.
  digitalWrite(PDRST, HIGH);
}

void read_adc()
{
  code_old = code_new; // save last measurement
 
  // read in adc data (sending out don't care bytes)
  // and store read data into four bytes */
  byte1 = SPI.transfer(0x00);
  byte2 = SPI.transfer(0x00);
  byte3 = SPI.transfer(0x00);
  byte4 = SPI.transfer(0x00);
  //save 20 bit of ADC reading
  code_new = (byte2 << 4) | (byte3 >>4);
  middle_byte = (byte1 << 4) | (byte2 >> 4);
  left_byte = byte1>>4;
  code_new = code_new | (middle_byte<<8) | (left_byte<<16);
  // differential weight between single measurments
  code_diff = code_new - code_old;
  weight = abs(code_diff*0.037);
  Serial.println(code_new,DEC);
  Serial.println(code_old,DEC);
  Serial.println(code_diff,DEC);
//  Serial.print("gram=");
//  Serial.println(weight,DEC);
  if(byte4 == 0x01 ){
      Serial.println( "serial transfer from the ADC was performed correctly");
  }
//    delay(1000);
//  byte1=byte1 | 0x80;
//  Serial.println(byte1,HEX);
//  first2Byte = word(byte1,byte2);
//  code_new = first2Byte;
//  code_new=code_new<<8;
//  code_new=(code_new | byte3);
  
  //check if SPI transmission OK
  
  
  //building 32 bit code from read bytes. first reset sign bit(MSB bit).
//   byte1 = byte1 & 0x7F;
//   first2Byte = word(byte1,byte2);
//   code_old=code_new;
//   code_new = first2Byte;
//   code_new=code_new<<8;
//   code_new = code_new + byte3;
//   code_diff = code_old - code_new;
//   Serial.print("code_old=");
//   Serial.println(code_old,DEC);
//   Serial.print("code_new=");
//   Serial.println(code_new,DEC);
//   Serial.print("code_diff=");
//   Serial.println(code_diff,DEC);
//   code_old_comp_2=(!code_old)+1;
//   code_new_comp_2=(!code_new)+1;
//   code_diff=(code_old_comp_2-code_new_comp_2);
//   Serial.print("code_new_comp_2=");
//   Serial.println(code_new_comp_2,DEC);
//   Serial.print("code_old_comp_2=");
//   Serial.println(code_old_comp_2,DEC);
//   Serial.print("code_diff=");
//   Serial.println(code_diff,DEC);
//  Serial.println(first2Byte,BIN);
//  Serial.println(code, BIN);
//  Serial.print("   ");
//  code = first2Byte<<8;
//  Serial.println(code, BIN);
//  code = code|byte3;  
//  Serial.println(code, BIN);
  }

// wait for DRDY to pass and to reach start-point of DOUT
void drdy_wait()
{
   delay(130);
}
