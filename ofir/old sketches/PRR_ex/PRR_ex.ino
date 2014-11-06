
void setup(){
  PRR = B11111111;
}
void loop(){
 PRR = B11111111;
 delay(5000);
 PRR = B00000000;
}
