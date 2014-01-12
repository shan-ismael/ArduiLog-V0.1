/* 
 ArduiLog v.01 http://rc-lab.fr
 MegunoLink users code - with english comments
 
 Short link : http://wp.me/p2z2tt-7u
 http://rc-lab.fr/arduilog-v-01-appareil-dacquisition-48-voies-analogiques
 
 Shanmugathas Vigneswara-Ismaël.
 
 */
 
 
 /*
 Thanks to MuxShield : http://mayhewlabs.com/arduino-mux-shield
 for multiplexer code.
 
 Thanks to Maurice Ribble 4-17-2008
 http://www.glacialwanderer.com/hobbyrobotics
 for DS1307 RTC code.
 
 Thanks to Edouard Chalouhb : http://www.edouard-chalhoub.com/
 to provide me a lab to test ArduiLog v0.1
 */


#include <Wire.h>
#include <SerialLCD.h>
#include <SoftwareSerial.h>
#include <math.h>

// Use software serial to prind on LCD display
SerialLCD slcd(11,12);

// Define real time clock adresse
#define DS1307_I2C_ADDRESS 0x68

//Define mutliplexer digital control pins
#define CONTROL0 5    
#define CONTROL1 4
#define CONTROL2 3
#define CONTROL3 2

int analog_input[48]; // Raw analog input of multiplexer
int an_disp_count = 0; // Integer used to count LCD display
char* input_unit[48]; // Your analog input unit configuration
char* input_label[48]; // Your analog input label/comment
float analog_mapped[48]; // Use equations to convert raw analog input to physical value

// Store time and date value from RTC module
byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;


void setup()
{
  //Open the serial port at 500000 bps
  Serial.begin(500000);

  //Open Serial LCD
  slcd.begin();

  // lcd backlight on
  slcd.backlight();

  // Openr Wire communication for RTC
  Wire.begin();

  //Set MUX control pins to output
  pinMode(CONTROL0, OUTPUT);
  pinMode(CONTROL1, OUTPUT);
  pinMode(CONTROL2, OUTPUT);
  pinMode(CONTROL3, OUTPUT);

  /* 
   Change these values to what you want to set your clock to.
   You probably only want to set your clock once and then remove
   the setDateDs1307 call.
   */

  /*
   second = 40;
   minute = 54;
   hour = 14;
   dayOfWeek = 5;
   dayOfMonth = 20;
   month = 12;
   year = 13;
   setDateDs1307(second, minute, hour, dayOfWeek, dayOfMonth, month, year);
   */


  // Set analogs inputs units  Megunolink and LCD display (Pa,°C,V,mA,A ....)
  input_unit[0]="";   // analog1 unit
  input_unit[1]="";   // analog2 unit
  input_unit[2]="";   // analog3 unit
  input_unit[3]="";   // analog4 unit
  input_unit[4]="";   // analog5 unit
  input_unit[5]="";   // analog6 unit
  input_unit[6]="";   // analog7 unit
  input_unit[7]="";   // analog8 unit
  input_unit[8]="";   // analog9 unit
  input_unit[9]="";   // analog10 unit
  input_unit[10]="";   // analog11 unit   
  input_unit[11]="";   // analog12 unit
  input_unit[12]="";   // analog13 unit
  input_unit[13]="";   // analog14 unit
  input_unit[14]="";   // analog15 unit
  input_unit[15]="";   // analog16 unit
  input_unit[16]="";   // analog17 unit
  input_unit[17]="";   // analog18 unit
  input_unit[18]="";   // analog19 unit
  input_unit[19]="";   // analog20 unit
  input_unit[20]="";   // analog21 unit
  input_unit[21]="";   // analog22 unit
  input_unit[22]="";   // analog23 unit
  input_unit[23]="";   // analog24 unit
  input_unit[24]="";   // analog25 unit
  input_unit[25]="";   // analog26 unit
  input_unit[26]="";   // analog27 unit
  input_unit[27]="";   // analog28 unit
  input_unit[28]="";   // analog29 unit
  input_unit[29]="";   // analog30 unit
  input_unit[30]="";   // analog31 unit
  input_unit[31]="";   // analog32 unit
  input_unit[32]="";   // analog33 unit
  input_unit[33]="";   // analog34 unit
  input_unit[34]="";   // analog35 unit
  input_unit[35]="";   // analog36 unit
  input_unit[36]="";   // analog37 unit
  input_unit[37]="";   // analog38 unit
  input_unit[38]="";   // analog39 unit
  input_unit[39]="";   // analog40 unit
  input_unit[40]="";   // analog41 unit
  input_unit[41]="";   // analog42 unit
  input_unit[42]="";   // analog43 unit
  input_unit[43]="";   // analog44 unit
  input_unit[44]="";   // analog45 unit
  input_unit[45]="";   // analog46 unit
  input_unit[46]="";   // analog47 unit
  input_unit[47]="";   // analog48 unit

  // Configure analogs labels for Megunolink (Cooling room 1, Motor 2, Testing something 8 ...)
  input_label[0]="";   // analog1 label
  input_label[1]="";   // analog2 label
  input_label[2]="";   // analog3 label
  input_label[3]="";   // analog4 label
  input_label[4]="";   // analog5 label
  input_label[5]="";   // analog6 label
  input_label[6]="";   // analog7 label
  input_label[7]="";   // analog8 label
  input_label[8]="";   // analog9 label
  input_label[9]="";   // analog10 label
  input_label[10]="";   // analog11 label   
  input_label[11]="";   // analog12 label
  input_label[12]="";   // analog13 label
  input_label[13]="";   // analog14 label
  input_label[14]="";   // analog15 label
  input_label[15]="";   // analog16 label
  input_label[16]="";   // analog17 label
  input_label[17]="";   // analog18 label
  input_label[18]=" ";   // analog19 label
  input_label[19]="";   // analog20 label
  input_label[20]="";   // analog21 label
  input_label[21]="";   // analog22 label
  input_label[22]="";   // analog23 label
  input_label[23]="";   // analog24 label
  input_label[24]="";   // analog25 label
  input_label[25]="";   // analog26 label
  input_label[26]="";   // analog27 label
  input_label[27]="";   // analog28 label
  input_label[28]="";   // analog29 label
  input_label[29]="";   // analog30 label
  input_label[30]="";   // analog31 label
  input_label[31]="";   // analog32 label
  input_label[32]="";   // analog33 label
  input_label[33]="";   // analog34 label
  input_label[34]="";   // analog35 label
  input_label[35]="";   // analog36 label
  input_label[36]="";   // analog37 label
  input_label[37]="";   // analog38 label
  input_label[38]="";   // analog39 label
  input_label[39]="";   // analog40 label
  input_label[40]="";   // analog41 label
  input_label[41]="";   // analog42 label
  input_label[42]="";   // analog43 label
  input_label[43]="";   // analog44 label
  input_label[44]="";   // analog45 label
  input_label[45]="";   // analog46 label
  input_label[46]="";   // analog47 label
  input_label[47]="";   // analog48 label




}



void loop()
{
  get_time(); // get time from RTC 
  read_analogs(); // read analog from multiplexer
  analogs_map();  // use your equationns, and map to convert resistance/voltage to physical value
  megunolink_datalog(); // print to Megunolink for datalogging
  megunolink_timeplot(); // print to Megunolink for timeplot
  megunolink_table(); // print to Megunolink for table of value
  lcd_print(); // display information on LCD
  delay(1000); // delay 1 sec to get a correct looping
}


void get_time(){
  // Get time from RTC module, you don't need to change this function
  getDateDs1307(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year);
}

void read_analogs(){
  // Read analog input for Multiplxer, you don't need to change this function

  //This for loop is used to scroll through and store the 16 inputs on the FIRST multiplexer
  for (int i=0; i<16; i++)
  {
    //The following 4 commands set the correct logic for the control pins to select the desired input
    //See the Arduino Bitwise AND Reference: http://www.arduino.cc/en/Reference/BitwiseAnd
    //See the Aruino Bitshift Reference: http://www.arduino.cc/en/Reference/Bitshift
    digitalWrite(CONTROL0, (i&15)>>3); 
    digitalWrite(CONTROL1, (i&7)>>2);  
    digitalWrite(CONTROL2, (i&3)>>1);  
    digitalWrite(CONTROL3, (i&1));     

    //Read and store the input value at a location in the array
    analog_input[i] = analogRead(0);
  }

  //This for loop is used to scroll through the SECOND multiplexer
  for (int i=16; i<32; i++)
  {
    digitalWrite(CONTROL0, (i&15)>>3); 
    digitalWrite(CONTROL1, (i&7)>>2);  
    digitalWrite(CONTROL2, (i&3)>>1);  
    digitalWrite(CONTROL3, (i&1));     
    analog_input[i] = analogRead(1);
  }

  //This for loop is used to scroll through the THIRD multiplexer
  for (int i=32; i<48; i++)
  {
    digitalWrite(CONTROL0, (i&15)>>3); 
    digitalWrite(CONTROL1, (i&7)>>2);  
    digitalWrite(CONTROL2, (i&3)>>1);  
    digitalWrite(CONTROL3, (i&1));     
    analog_input[i] = analogRead(2);
  }    
}

void analogs_map(){
  // Use this function to convert reading value to physical value

  // Voltage conversion, convert all analogs inputs from [0;1023] numeric to [0;24.5]V
  for (int i=0; i<48; i++){
    analog_mapped[i]= float_map(analog_input[i], 0, 1023, 0, 24.5); 
  }

  /* 
   Use this code to get resistance value 
   if you use thermistance for photosensor, 
   temperature etc... This code convert 
   mesured voltage to resistance value.
   
   float resistor_value;
   float resistor_value = ((-4.9*(analog_mapped[i] -5))/(analog_mapped[i] ))*1000;
   */


  /* 
   Use this code if you are 
   using TMP36 sensor, to convert mesured 
   voltage to temperature in °C
   
   float temperature_c;
   temperature_c = ( analog_mapped[i]*1000 - 500) / 10 
   */



}


void megunolink_datalog(){
  // Send values in message format to MegunoLink, to export value in *.txt format (to change to *.csv format)

  Serial.print("{MESSAGE|data|");
  Serial.print(hour, DEC);
  Serial.print(":");
  Serial.print(minute, DEC);
  Serial.print(":");
  Serial.print(second, DEC);
  Serial.print(";");

  for (int i=0; i<48; i++)
  {
    Serial.print(analog_mapped[i]);
    Serial.print(";");
  }

  Serial.println("}");
}

void megunolink_timeplot(){
  // Send values to MegunoLink to plot 

    for (int i=0; i<48; i++){
    Serial.print("{TIMEPLOT:");
    Serial.print("Analog"); // channel name
    Serial.print(i+1);        // channel number
    Serial.print("|data|");
    Serial.print(input_unit[i]);
    Serial.print(":");
    Serial.print("k_n");    // graphique configuration
    Serial.print("|T|");
    Serial.print(analog_mapped[i],3);  // data value
    Serial.println("}");
  }
}


void megunolink_table(){  
  // Send values to MegunoLink to get table

    for (int i=0; i<48; i++){
    Serial.print("{TABLE");
    Serial.print("|SET|");
    Serial.print("Analog");
    Serial.print(i+1);
    Serial.print("|");
    Serial.print(analog_mapped[i],3);
    Serial.print("|");
    Serial.print(input_unit[i]);
    Serial.print("  //  ");
    Serial.print(input_label[i]);
    Serial.println("}");
  }

}

void lcd_print(){
  // Display values on LCD

  slcd.clear();
  slcd.setCursor(0,0);
  slcd.print("Heure:");

  if (hour<0b1010) {
    slcd.setCursor(7,0);
    slcd.print("0");  
    slcd.setCursor(8,0);
    slcd.print(hour, DEC);
  }
  else{
    slcd.setCursor(7,0);
    slcd.print(hour, DEC);
  }

  slcd.setCursor(9,0);
  slcd.print(":");

  if (minute<0b1010) {
    slcd.setCursor(10,0);
    slcd.print("0");
    slcd.setCursor(11,0);
    slcd.print(minute, DEC);
  }
  else{
    slcd.setCursor(10,0);
    slcd.print(minute, DEC);
  }

  slcd.setCursor(12,0);
  slcd.print(":");

  if (second<0b1010) {
    slcd.setCursor(13,0);
    slcd.print("0");
    slcd.setCursor(14,0);
    slcd.print(second, DEC);
  }
  else{
    slcd.setCursor(13,0);
    slcd.print(second, DEC);
  }

  an_disp_count=an_disp_count+1;

  if (an_disp_count == 1){
    slcd.setCursor(0,1);
    slcd.print("A1:");
    slcd.setCursor(3,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);
    slcd.setCursor(9,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 2){
    slcd.setCursor(0,1);
    slcd.print("A2:");
    slcd.setCursor(3,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);
    slcd.setCursor(9,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 3){
    slcd.setCursor(0,1);
    slcd.print("A3:");
    slcd.setCursor(3,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);
    slcd.setCursor(9,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 4){
    slcd.setCursor(0,1);
    slcd.print("A4:");
    slcd.setCursor(3,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);
    slcd.setCursor(9,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 5){
    slcd.setCursor(0,1);
    slcd.print("A5:");
    slcd.setCursor(3,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);
    slcd.setCursor(9,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 6){
    slcd.setCursor(0,1);
    slcd.print("A6:");
    slcd.setCursor(3,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);
    slcd.setCursor(9,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 7){
    slcd.setCursor(0,1);
    slcd.print("A7:");
    slcd.setCursor(3,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);
    slcd.setCursor(9,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 8){
    slcd.setCursor(0,1);
    slcd.print("A8:");
    slcd.setCursor(3,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);
    slcd.setCursor(9,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 9){
    slcd.setCursor(0,1);
    slcd.print("A9:");
    slcd.setCursor(3,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);
    slcd.setCursor(9,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 10){
    slcd.setCursor(0,1);
    slcd.print("A10:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 11){
    slcd.setCursor(0,1);
    slcd.print("A11:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 12){
    slcd.setCursor(0,1);
    slcd.print("A12:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 13){
    slcd.setCursor(0,1);
    slcd.print("A13:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 14){
    slcd.setCursor(0,1);
    slcd.print("A14:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 15){
    slcd.setCursor(0,1);
    slcd.print("A15:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 16){
    slcd.setCursor(0,1);
    slcd.print("A16:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 17){
    slcd.setCursor(0,1);
    slcd.print("A17:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 18){
    slcd.setCursor(0,1);
    slcd.print("A18:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 19){
    slcd.setCursor(0,1);
    slcd.print("A19:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 20){
    slcd.setCursor(0,1);
    slcd.print("A20:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 21){
    slcd.setCursor(0,1);
    slcd.print("A21:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 22){
    slcd.setCursor(0,1);
    slcd.print("A22:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 23){
    slcd.setCursor(0,1);
    slcd.print("A23:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 24){
    slcd.setCursor(0,1);
    slcd.print("A24:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 25){
    slcd.setCursor(0,1);
    slcd.print("A25:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 26){
    slcd.setCursor(0,1);
    slcd.print("A26:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 27){
    slcd.setCursor(0,1);
    slcd.print("A27:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 28){
    slcd.setCursor(0,1);
    slcd.print("A28:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 29){
    slcd.setCursor(0,1);
    slcd.print("A29:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 30){
    slcd.setCursor(0,1);
    slcd.print("A30:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 31){
    slcd.setCursor(0,1);
    slcd.print("A31:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 32){
    slcd.setCursor(0,1);
    slcd.print("A32:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }
  if (an_disp_count == 33){
    slcd.setCursor(0,1);
    slcd.print("A33:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 34){
    slcd.setCursor(0,1);
    slcd.print("A34:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 35){
    slcd.setCursor(0,1);
    slcd.print("A35:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 36){
    slcd.setCursor(0,1);
    slcd.print("A36:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 37){
    slcd.setCursor(0,1);
    slcd.print("A37:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 38){
    slcd.setCursor(0,1);
    slcd.print("A38:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 39){
    slcd.setCursor(0,1);
    slcd.print("A39:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 40){
    slcd.setCursor(0,1);
    slcd.print("A40:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 41){
    slcd.setCursor(0,1);
    slcd.print("A41:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 42){
    slcd.setCursor(0,1);
    slcd.print("A42:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 43){
    slcd.setCursor(0,1);
    slcd.print("A43:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 44){
    slcd.setCursor(0,1);
    slcd.print("A44:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }
  if (an_disp_count == 45){
    slcd.setCursor(0,1);
    slcd.print("A45:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 46){
    slcd.setCursor(0,1);
    slcd.print("A46:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 47){
    slcd.setCursor(0,1);
    slcd.print("A47:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }

  if (an_disp_count == 48){
    slcd.setCursor(0,1);
    slcd.print("A48:");
    slcd.setCursor(4,1);
    SLCDprintFloat(analog_mapped[an_disp_count-1],2);    
    an_disp_count =0;
    slcd.setCursor(10,1);
    slcd.print(input_unit[an_disp_count-1]);
  }
}


byte decToBcd(byte val)
{
  return ( (val/10*16) + (val%10) );
}

// Convert binary coded decimal to normal decimal numbers
byte bcdToDec(byte val)
{
  return ( (val/16*10) + (val%16) );
}

void setDateDs1307(
byte second,        // 0-59
byte minute,        // 0-59
byte hour,          // 1-23
byte dayOfWeek,     // 1-7
byte dayOfMonth,    // 1-28/29/30/31
byte month,         // 1-12
byte year)  // 0-99

{
  Wire.beginTransmission(DS1307_I2C_ADDRESS);
  Wire.write(0);
  Wire.write(decToBcd(second));    // 0 to bit 7 starts the clock
  Wire.write(decToBcd(minute));
  Wire.write(decToBcd(hour));      // If you want 12 hour am/pm you need to set
  // bit 6 (also need to change readDateDs1307)
  Wire.write(decToBcd(dayOfWeek));
  Wire.write(decToBcd(dayOfMonth));
  Wire.write(decToBcd(month));
  Wire.write(decToBcd(year));
  Wire.endTransmission();
}

// Gets the date and time from the ds1307
void getDateDs1307(
byte *second,
byte *minute,
byte *hour,
byte *dayOfWeek,
byte *dayOfMonth,
byte *month,
byte *year)
{
  // Reset the register pointer
  Wire.beginTransmission(DS1307_I2C_ADDRESS);
  Wire.write(0);
  Wire.endTransmission();

  Wire.requestFrom(DS1307_I2C_ADDRESS, 7);

  // A few of these need masks because certain bits are control bits
  *second     = bcdToDec(Wire.read() & 0x7f);
  *minute     = bcdToDec(Wire.read());
  *hour       = bcdToDec(Wire.read() & 0x3f);  // Need to change this if 12 hour am/pm
  *dayOfWeek  = bcdToDec(Wire.read());
  *dayOfMonth = bcdToDec(Wire.read());
  *month      = bcdToDec(Wire.read());
  *year       = bcdToDec(Wire.read());
}


void SLCDprintFloat(double number, uint8_t digits) 

/* 
 Serial LCD don't support float display, 
 use this function if you want display flaot
 SLCDprintFloat(float_value,units) 
 */

{ 
  // Handle negative numbers
  if (number < 0.0)
  {
    slcd.print('-');
    number = -number;
  }

  // Round correctly so that slcd.print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i=0; i<digits; ++i)
    rounding /= 10.0;

  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  float remainder = number - (float)int_part;
  slcd.print(int_part , DEC); // base DEC

    // Print the decimal point, but only if there are digits beyond
  if (digits > 0)
    slcd.print("."); 

  // Extract digits from the remainder one at a time
  while (digits-- > 0)
  {
    remainder *= 10.0;
    float toPrint = float(remainder);
    slcd.print(toPrint , DEC); //base DEC
    remainder -= toPrint; 
  } 
}

float float_map(float x, float in_min, float in_max, float out_min, float out_max)
{
  /* 
   Use this function if you want map values, x = input reading value
   x1 -> x2 >>> y1 -> y2
   */
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}







