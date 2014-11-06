
/*
read Adxl345 Xg,Yg,Zg accelration and send data to PC through Bluetooth serial
port.
Gyro connectivity:
pin13 -SCl,pin12 - SDO,pin11 -SDA, pin10-CS, VCC=3.3V, GND
BT HC-05 connectivity:
RX - TXD,TX-RXD, VCC=5V,GND
Bluetooth baudrate = 9600
*/

 //Add the SPI library so we can communicate with the ADXL345 sensor
#include <SPI.h>

//Assign the Chip Select signal to pin 10.
int CS=10;

//This is a list of some of the registers available on the ADXL345.
//To learn more about these and the rest of the registers on the ADXL345, read the datasheet!
char POWER_CTL = 0x2D;    //Power Control Register
char DATA_FORMAT = 0x31;
char DATAX0 = 0x32;   //X-Axis Data 0
char DATAX1 = 0x33;   //X-Axis Data 1
char DATAY0 = 0x34;   //Y-Axis Data 0
char DATAY1 = 0x35;   //Y-Axis Data 1
char DATAZ0 = 0x36;   //Z-Axis Data 0
char DATAZ1 = 0x37;   //Z-Axis Data 1

//This buffer will hold values read from the ADXL345 registers.
unsigned char values[10];
//These variables will be used to hold the x,y and z axis accelerometer values.
int x,y,z;
//These variables will be used to hold the x,y and z axis accelerometer values in G's values.
float xg,yg,zg;

float r;//store the magnitued of xg,yg,zg
float p;//store tilt angle
float roll;//strore roll angle


void setup(){ 
  //Initiate an SPI communication instance.
  SPI.begin();
  //Configure the SPI connection for the ADXL345.
  SPI.setDataMode(SPI_MODE3);
  
  //Create a serial connection between Bluetooth to PC (Also to serial monitor).
  Serial.begin(9600);
  
  //Set up the Chip Select pin to be an output from the Arduino.
  pinMode(CS, OUTPUT);
  //Before communication starts, the Chip Select pin needs to be set high.
  digitalWrite(CS, HIGH);
  
  //Put the ADXL345 into +/- 4G range by writing the value 0x01 to the DATA_FORMAT register.
  writeRegister(DATA_FORMAT, 0x01);
  //Put the ADXL345 into Measurement Mode by writing 0x08 to the POWER_CTL register.
  writeRegister(POWER_CTL, 0x08);  //Measurement mode  
}

void loop(){
  //Reading 6 bytes of data starting at register DATAX0 will retrieve the x,y and z acceleration values from the ADXL345.
  //The results of the read operation will get stored to the values[] buffer.
  readRegister(DATAX0, 6, values);

  //The ADXL345 gives 10-bit acceleration values, but they are stored as bytes (8-bits). To get the full value, two bytes must be combined for each axis.
  //The X value is stored in values[0] and values[1].
  x = ((int)values[1]<<8)|(int)values[0];
  //The Y value is stored in values[2] and values[3].
  y = ((int)values[3]<<8)|(int)values[2];
  //The Z value is stored in values[4] and values[5].
  z = ((int)values[5]<<8)|(int)values[4];
  
  //Convert the accelerometer value to G's. 
  //With 10 bits measuring over a +/-4g range we can find how to convert by using the equation:
  // Gs = Measurement Value * (G-range/(2^10)) or Gs = Measurement Value * (8/1024)
  xg = x * 0.0078;
  yg = y * 0.0078;
  zg = z * 0.0078;
  
  //Print the results to the terminal.
 // Serial.println(x, DEC);
  //Serial.println(y, DEC);
  //Serial.println(z, DEC);      
  //delay(2000); 
  
  //Print the results in G's values to the terminal.
  //Serial.println(xg, DEC);
  //Serial.println(yg, DEC);
  //Serial.println(zg, DEC );      
  
  //calculating tilt angle between gravitational vector and ADXL Z axis
  r=sqrt(pow(xg,2)+pow(yg,2)+pow(zg,2));
  p=acos(zg/r)*(180/PI);
 // Serial.println("tilt" ); 
  //Serial.println(p, DEC ); 
   //calculating the roll angle , rotaion angle around Vector X
  roll = atan2(yg,sqrt(pow(xg,2)+pow(zg,2)));
  roll=roll*(180/PI);
  
 //Serial.println("roll" ); 
  //Serial.println(roll, DEC );
   
   Serial.print(float(p));
   delay(100); 
}


//This function will write a value to a register on the ADXL345.
//Parameters:
//  char registerAddress - The register to write a value to
//  char value - The value to be written to the specified register.
void writeRegister(char registerAddress, char value){
  //Set Chip Select pin low to signal the beginning of an SPI packet.
  digitalWrite(CS, LOW);
  //Transfer the register address over SPI.
  SPI.transfer(registerAddress);
  //Transfer the desired register value over SPI.
  SPI.transfer(value);
  //Set the Chip Select pin high to signal the end of an SPI packet.
  digitalWrite(CS, HIGH);
}

//This function will read a certain number of registers starting from a specified address and store their values in a buffer.
//Parameters:
//  char registerAddress - The register addresse to start the read sequence from.
//  int numBytes - The number of registers that should be read.
//  char * values - A pointer to a buffer where the results of the operation should be stored.
void readRegister(char registerAddress, int numBytes, unsigned char * values){
  //Since we're performing a read operation, the most significant bit of the register address should be set.
  char address = 0x80 | registerAddress;
  //If we're doing a multi-byte read, bit 6 needs to be set as well.
  if(numBytes > 1)address = address | 0x40;
  //Set the Chip select pin low to start an SPI packet.
  digitalWrite(CS, LOW);
  //Transfer the starting register address that needs to be read.
  SPI.transfer(address);
  //Continue to read registers until we've read the number specified, storing the results to the input buffer.
  for(int i=0; i<=numBytes; i++){
          values[i] = SPI.transfer(0x00);
  }
  //Set the Chips Select pin high to end the SPI packet.
  digitalWrite(CS, HIGH);
}

