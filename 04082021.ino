/*=============================================================*/
/*=============Communicate with Stellarium=====================*/
//input form stellarium
char inputChar[20];
//init RAHH,RAMM,RASS to visualize the star pointer;
char RAHH[3]="12",RAMM[3]="00",RASS[3]="00";
//init DECSign,DECDeg,DECMM,DECSS to visualize the star pointer;
char DECSign[2]="-",DECDeg[3]="12",DECMM[3]="00",DECSS[3]="00";
char txRA[10];
char txDEC[11];
char signDEC;
// store the targe data from Stellarium
String RAtargetString, DECtargetString;
//convert RA/DEC into degree
float RADegree, DECDegree;

/*-------------------------------
---------OLED setting-------------
================================*/
#define I2C_ADDRESS 0x3C
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"
#define I2C_ADDRESS 0x3C
SSD1306AsciiAvrI2c oled;

/*=============================================================*/
/*====================SPI readout setting=========================*/
#include "SPI.h" // necessary library
#define DATAOUT 11//MOSI
#define DATAIN  12//MISO
#define SPICLOCK  13//sck
#define SLAVESELECT 10//ss

byte clr;
//data buffer
char buffer [128];
byte spi_output_data;
char gpsStream;

char spi_transfer(char data)
{
  SPDR = data;                    // Start the transmission
  while (!(SPSR & (1<<SPIF)))     // Wait the end of the transmission
  {
  };
  return SPDR;                    // return the received byte
}

byte read()
{
  int data;
  digitalWrite(SLAVESELECT,LOW);
  data = spi_transfer(0x00); //get data byte
  digitalWrite(SLAVESELECT,HIGH); //release chip, signal end transfer
  return data;
}

/*=============================================================*/
/*====================GPS time setting=========================*/
#include <TinyGPS++.h>
TinyGPSPlus gps;
int Year = 2021;                            // Date (Year)
int Month = 04;                             // Date (Month)
int Day  = 04;                              // Date (Day) 
int timeHH=16;                              // Time (Hours)
int timeMM=16;                              // Time (Minutes)
int timeSS=0;                               // Time (Seconds)
float Latitude;                             // Latitude (signed degrees)
float Longitude;                            // Longitude (signed degrees)
#define pi 3.14159265
#define rad2deg(X) ((X)/pi*180)             //Rad -->> degree
#define deg2rad(X) ((X)/180*pi)             //Degree-->>rad
float Timenow;
float Alt = 0.0, Az = 0.0; // Target star Altitude and Azimuth
float ra; //These variables are used in the calculations, float/float/int/byte depending on the type of number needed.
float dec;
float lst;

/*=============================================================*/
/*======================Servo setting==========================*/
#include <Servo.h>    // used to control the servos
Servo servo_alt, servo_az;
float object_alt = 0.0; // this is north and horizontal
float object_az  = 0.0;
float position_alt = object_alt;
float position_az = object_az;
int slew_speed = 10; // this is a delay in ms
int received_az = position_az, received_alt = position_alt;

void setup() {
  //set Main Baud Rate
  Serial.begin(9600);
  //setup SPI communicates
  pinMode(DATAOUT, OUTPUT);
  pinMode(DATAIN, INPUT);
  pinMode(SPICLOCK,OUTPUT);
  pinMode(SLAVESELECT,OUTPUT);
  digitalWrite(SLAVESELECT,HIGH); //disable device
  // SPCR = 01010000
  //interrupt disabled,spi enabled,msb 1st,master,clk low when idle,
  //Enble SPI Master, SET 
  SPCR = (1<<SPE)|(1<<MSTR)| (0<<CPOL)| (1<<SPR1) | (1<<SPR0);
  //clear any pending SPI Interrupt.
  // by reading the SPSR (SPI Status Register), and the SPDR (SPI Data Register) any pending SPI interrupt is cleared.
  clr=SPSR;
  clr=SPDR; 
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
  oled.setFont(System5x7);
  oled.clear();
  servo_az.attach(8); // azimut servo is on pin D8
  servo_alt.attach(7); // altitude servo is on pin D7
  // set init position (north/horizontal)
  set_azimut((int)position_az);
  set_altitude((int)position_alt);  
}

void loop() {
  spi_output_data = read();
      delay(100); //pause for readability
      if (gps.encode((char)spi_output_data)){
        if (gps.location.isValid()){
          Latitude= gps.location.lat();
          Longitude= gps.location.lng();
          oled.setCursor(0, 2);
          oled.print(Longitude);
          }else{
          Latitude=48.5216; 
          Longitude=9.0576;
        }
       if (gps.date.isValid()){
          Month= gps.date.month();
          Day= gps.date.day();
          Year= gps.date.year();
        }
       if (gps.time.isValid()){
          timeHH= gps.time.hour();
          timeMM= gps.time.minute();
          timeSS= gps.time.second();
        } 
       Timenow = ((float)timeHH + ((((float)timeSS / 60.0) + (float)timeMM) / 60.0));
        oled.setCursor(0, 8);
        oled.print(Timenow);
  }
  
  // communicate with Stellarium
  if(Serial.available()){
    int i= 0;
    //read out the command from Stellarium
    inputChar[i++]= Serial.read();
    oled.setCursor(0, 1);
    oled.print(inputChar);
    while((inputChar[i++]=Serial.read()) != '#'){
      delay(5);
    }
    ////ask telescope present RA #:GR#
    if(inputChar[1]==':' && inputChar[2]=='G' && inputChar[3]=='R' && inputChar[4]=='#'){
      Serial.print(txRA);
    }
    //ask telescope present DEC #:GD#
    if(inputChar[1]==':' && inputChar[2]=='G' && inputChar[3]=='D' && inputChar[4]=='#'){
      Serial.print(txDEC);
    }
    // get ":Q#"
    if(inputChar[1]==':' && inputChar[2]=='Q' && inputChar[3]=='#'){
      //Do nothing with Stellarium
      //up date the GPS data
    }
    //arduino get target RA when user type ctrl+1 :Sr 10:10:10#   
    if(inputChar[0]==':' && inputChar[1]=='S' && inputChar[2]=='r'){
      // answer 1 continue
      Serial.print("1");
      // get the string
      RAHH[0]=inputChar[3];
      RAHH[1]=inputChar[4];
      RAMM[0]=inputChar[6];
      RAMM[1]=inputChar[7];
      RASS[0]=inputChar[9];
      RASS[1]=inputChar[10];
      RAtargetString="RA: ";
      RAtargetString+=RAHH;
      RAtargetString+="h ";
      RAtargetString+=RAMM;
      RAtargetString+="m ";
      RAtargetString+=RASS;
      RAtargetString+="s";
    }
    //arduino get target DEC when user type ctrl+1 #:Sd +76:24:55# 
    if(inputChar[0]==':' && inputChar[1]=='S' && inputChar[2]=='d'){
      // answer 1 continue
      Serial.print("1");
      // get the string
      DECSign[0]=inputChar[3];
      DECDeg[0]=inputChar[4];
      DECDeg[1]=inputChar[5];
      DECMM[0]=inputChar[7];
      DECMM[1]=inputChar[8];
      DECSS[0]=inputChar[10];
      DECSS[1]=inputChar[11];
      DECtargetString="DEC: ";
      DECtargetString+=DECSign;
      DECtargetString+=DECDeg;
      DECtargetString+="deg ";
      DECtargetString+=DECMM;
      DECtargetString+="' ";
      DECtargetString+=DECSS;
      DECtargetString+="''";
    }
    //:MS# Slew to Target Object
    if(inputChar[0]==':' && inputChar[1]=='M' && inputChar[2]=='S' && inputChar[3]=='#'){    
      //RETURNS 0 means OK
      Serial.print("0");  
      //update RADEC string into float
      getRADEC_to_Degree();
      RADEC_2_AZALT();   
      AZALT_TO_DECRA();
      set_azimut((int)object_az);
      set_altitude((int)object_alt);
    }
  }
}

//Convert string to float
void getRADEC_to_Degree(){
  //RAHH in char, get RA in Degree which is hour *15, one hour = 15 degrees
  RADegree = (atof(RAHH) + (atof(RAMM)/60) +(atof(RASS)/3600)) *15;   // degrees in decimal
  //get DEC in Degree
  DECDegree = (atof(DECDeg) + atof(DECMM)/60 + atof(DECSS)/3600);  // degrees in decimal
  //Convert to negative when it is negative
  if (DECSign[0] == '-'){
    DECDegree = 0- DECDegree;
  }
}

//Equatorial to horizon coordinate conversion
void RADEC_2_AZALT(){
  /*===========Now Time calculation ============
  ================================================*/
  //base on 2011 - Practical Astronomy with your Calculator or Spreadsheet_4thEd - DUFFETT-SMITH
  
  //===*1*=== julian date at 0h 
  float Today =  float(Day); //longitude in the west need to minus 1, today like 2009 June  (i.e. 6 pm on 19 June is 19.75).
  if(Month == 1 || Month == 2){
    Month = Month + 12;
    Year = Year - 1;
  }
  int a = floor(Year / 100);                   // floor to get trunc
  int b = 2 - a + floor(a / 4);
  float jd = floor(365.25 * (Year + 4716)) + floor(30.6001 * (Month + 1)) + Today + b - 1524.5;
  
  //===*2*=== UT_TO_GST
  float s = jd - 2451545.0;
  float t = s / 36525.0;
  float T0 = 6.697374558 + (2400.051336 * t) + (0.000025862 * t * t);
  //Reduce the result to the range 0 to 24 by adding or subtracting multiples of 24.
  float n1 = floor (T0 /24);
         T0 = T0 - (n1 * 24);
  float h1 = (Timenow * 1.002737909);
  //reduce to the range 0 to 24 if necessary by subtracting or adding 24.
  float n2 = floor((T0 + h1)/ 24.0);
  float gst = (T0 + h1)- (n2 * 24.0);
  
  //===*3*=== GST_TO_LST
  //convers Greenwich time to Local sidereal time
  float geolon = (Longitude/15);  //Convert the geographical longitude in degrees to its equivalent in hours by dividing by 15.
  
  if ((Longitude * -1) > 0){
        gst = gst -geolon;
  }
  else {
    gst = gst + geolon;
  }
  if (gst > 24){
    lst = gst - 24;
  }
  else if ((gst * -1)>0){
    lst = gst +24;
  }
  else{
    lst = gst;
  }
  
  //===*4*=== Calculation hour angle
  //Hour Angle of star = Local Sidereal Time- Right Ascension of star
 float Ha = lst - (RADegree/15);
 Ha = Ha*15; // Multiply by 15 to convert Ha to degrees
 if (Ha < 0){
 Ha = 360 + Ha;
 }
/*===========ALT AZ calculations ============*/
 //dec = ((DECDegree / 360) * (2 * PI));
 Alt=rad2deg(asin(sin(deg2rad(DECDegree))*sin(deg2rad(Latitude))+cos(deg2rad(DECDegree))*cos(deg2rad(Latitude))*cos(deg2rad(Ha))));
 Az=rad2deg(acos((sin(deg2rad(DECDegree)) -(sin(deg2rad(Alt))*sin(deg2rad(Latitude))))/(cos(deg2rad(Alt)) * cos(deg2rad(Latitude)))));
 //if sin(HA) is positive, the angle AZ is 360 - AZ
 if (rad2deg(sin(deg2rad(Ha))) > 0) {
 Az= 360 - Az;
 }
 object_alt = Alt;
 oled.setCursor(0, 3);
 oled.print(Alt);
 object_az = Az;
 oled.setCursor(0, 4);
 oled.print(Az);
}
void set_altitude(int new_alt)
{
  // check in which direction we have to go
  if (new_alt > position_alt) {
    // to slow down, we do 1 deg steps and a small pause
    for (int tmp_alt = position_alt; tmp_alt <= new_alt; tmp_alt += 1)
    {
      position_alt = tmp_alt;                             //update the position
      servo_alt.write(map(position_alt, 0, 90, 180, 90));//remap and write the new position
      delay(slew_speed);                                 //wait
    } 
  } else {
    for (int tmp_alt = position_alt; tmp_alt >= new_alt; tmp_alt -= 2)
    {
      position_alt = tmp_alt;                            //update the position
      servo_alt.write(map(position_alt, 0, 90, 180, 90));//remap and write the new position
      delay(slew_speed);                                 //wait
    }
  }
}
void set_azimut(int new_az)
{  
  // check in which direction we have to go
  if (new_az > position_az) {
    // to slow down, we do 1 deg steps and a small pause
    for (int tmp_az = position_az; tmp_az <= new_az; tmp_az += 2)
    {
      position_az = tmp_az;         //update the position
      servo_az.write(position_az/2);//write the new position
      delay(slew_speed);            //wait
    }   
  } else {
    for (int tmp_az = position_az; tmp_az >= new_az; tmp_az -= 2)
    {
      position_az = tmp_az;         //update the position
      servo_az.write(position_az/2);//write the new position
      delay(slew_speed);            //wait
    }
  }
}

//Alt Az to Ra Dec conversion
void AZALT_TO_DECRA(){
  float DEC_Cal;
  float HA_Cal;
  float Ra_Cal;
  DEC_Cal = rad2deg(asin(sin(deg2rad(Alt))*sin(deg2rad(Latitude))+cos(deg2rad(Alt))*cos(deg2rad(Latitude))*cos(deg2rad(Az))));
  HA_Cal = rad2deg(acos((sin(deg2rad(Alt)) - sin(deg2rad(Latitude))*sin(deg2rad(DEC_Cal)))/(cos(deg2rad(Latitude))*cos(deg2rad(DEC_Cal)))));
  if (rad2deg(sin(deg2rad(Az))) > 0){
    HA_Cal = 360 - HA_Cal;
  }
  Ra_Cal = lst*15 - HA_Cal;
  while(Ra_Cal >= 360){
    Ra_Cal =Ra_Cal - 360;
  }
  while(Ra_Cal <= 0){
    Ra_Cal =Ra_Cal + 360;
  }
  int RAHH_Cal = Ra_Cal/15;
  int RAMM_Cal = ((Ra_Cal-(RAHH_Cal*15))*60)/15;
  int RASS_Cal = ((Ra_Cal-(RAHH_Cal*15 + (RAMM_Cal*15)/60))*3600)/15;
  int DECDeg_Cal = int(DEC_Cal);
  int DECMM_Cal = int((DEC_Cal-DECDeg_Cal)*60);
  int DECSS_Cal = (DEC_Cal-DECDeg_Cal-DECMM_Cal/60)*3600;
  (DEC_Cal < 0) ? signDEC = 45 : signDEC = 43;
  sprintf(txRA, "%02d:%02d:%02d#", RAHH_Cal,RAMM_Cal,RASS_Cal);
  sprintf(txDEC, "%c%02d%c%02d:%02d#", signDEC, DECDeg_Cal, 223, DECMM_Cal, DECSS_Cal);
  oled.setCursor(0, 6);
  oled.print(DEC_Cal);
  oled.setCursor(50, 6);
  oled.print(Ra_Cal);
}
