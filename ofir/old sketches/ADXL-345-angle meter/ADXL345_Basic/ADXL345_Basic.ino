//Add the SPI library so we can communicate with the ADXL345 sensor
#include <SPI.h>
//#include <math.h>


//Assign the Chip Select signal to pin 10.
int CS=10;
//char G_EARTH = 9.81;
//This is a list of some of the registers available on the ADXL345.
//To learn more about these and the rest of the registers on the ADXL345, read the datasheet!
char POWER_CTL = 0x2D;	//Power Control Register - entering standby mode
char DATA_FORMAT = 0x31;
char DATAX0 = 0x32;	//X-Axis Data 0
char DATAX1 = 0x33;	//X-Axis Data 1
char DATAY0 = 0x34;	//Y-Axis Data 0
char DATAY1 = 0x35;	//Y-Axis Data 1
char DATAZ0 = 0x36;	//Z-Axis Data 0
char DATAZ1 = 0x37;	//Z-Axis Data 1

//This buffer will hold values read from the ADXL345 registers.
char values[10];
//These variables will be used to hold the x,y and z axis accelerometer values.
double x,y,z;

//float x,y,z, pitch, roll;
double Gpx,Gpy,Gpz,u=0.5;

void setup(){ 
  //Initiate an SPI communication instance.
  SPI.begin();
  //Configure the SPI connection for the ADXL345.
  SPI.setDataMode(SPI_MODE3);
  //Create a serial connection to display the data on the terminal.
  Serial.begin(9600);
  Serial.println('test');
  //Set up the Chip Select pin to be an output from the Arduino.
  pinMode(CS, OUTPUT);
  //Before communication starts, the Chip Select pin needs to be set high.
  digitalWrite(CS, HIGH);
  
  //Put the ADXL345 into +/- 4G range by writing the value 0x01 to the DATA_FORMAT register.
  writeRegister(DATA_FORMAT, 0x01); // No Self-test, 4 wire SPI,10-bit mode,right-justified mode with sign extension,
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
  
 //g force per axis
//    Gpx = x* 0.0078;
//    Gpy = y * 0.0078;
//    Gpz = z * 0.0078;
//    
//Serial.println("Gpx+Gpy+Gpz");
//Serial.println(sqrt(Gpx*Gpx+Gpy*Gpy+Gpz*Gpz),DEC);
Serial.print("mag");
Serial.println(sqrt(x*x+y*y+z*z),DEC);
//Serial.print("X  ");
//Serial.println(x,DEC);
//Serial.print("Y  ");
//Serial.println(y,DEC);
//Serial.print("Z  ");
//Serial.println(z,DEC);

//Serial.println("x+y+z");
//Serial.println(x+y+z);
//Roll & Pitch Equations
//roll = (atan2(-y, z)*180.0)/PI;
//pitch = (atan2(x, sqrt(y*y + z*z))*180.0)/PI; 
  
////pitch theta angle
Serial.println("roll");
if(Gpz>=0){
 Serial.println(atan2(-x, sqrt(z*z + y*y))*180/PI,DEC);
}
else{
 Serial.println(atan2(-x, -sqrt(z*z + y*y))*180/PI,DEC);
}
//roll theta angle
Serial.println("pitch");
Serial.println(atan2(y, sqrt(x*x + z*z))*180/PI,DEC);

  
////Print the results to the terminal.
//Serial.println("Gpx");
//Serial.println(Gpx, DEC);
//Serial.println("Gpy");
//Serial.print(Gpy, DEC);
//Serial.println("Gpz");
//Serial.println(Gpz, DEC);

delay(5000); 

//  if ( x>=0 && y>=0){
  // Serial.print( atan2(x,y) * 180 / PI, DEC);
  // Serial.print( asin(x) * 180 / PI, DEC);
//    Serial.println("roll");
//    Serial.println(atan2(y, z)*180/PI,DEC);
//   Serial.println("pitch");
//    Serial.println(atan2(x, sqrt(y*y + z*z))*180/PI,DEC);
//  }
//  if( x>=0 && y<0){
//    Serial.print( 180 + (atan2(x,y) * 180 / PI),DEC);
//  }
//  if( x<=0 && y>=0){
//    Serial.print( (atan2(x,y) * 180 / PI),DEC);
//  }
//   if( x<=0 && y<=0){
   // Serial.print( (atan2(x,y) * 180 / PI)-180, DEC);
//  }
//   delay(3000); 
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
void readRegister(char registerAddress, int numBytes, char * values){
  //Since we're performing a read operation, the most significant bit of the register address should be set.
  char address = 0x80 | registerAddress;
  //If we're doing a multi-byte read, bit 6 needs to be set as well.
  if(numBytes > 1)address = address | 0x40;
  
  //Set the Chip select pin low to start an SPI packet.
  digitalWrite(CS, LOW);
  //Transfer the starting register address that needs to be read.
  SPI.transfer(address);
  //Continue to read registers until we've read the number specified, storing the results to the input buffer.
  for(int i=0; i<numBytes; i++){
    values[i] = SPI.transfer(0x00);
  }
  //Set the Chips Select pin high to end the SPI packet.
  digitalWrite(CS, HIGH);
}
