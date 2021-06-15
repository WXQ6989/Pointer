/**
 * Define the global variable for the Communicate between Stellarium and Arduino
 *
 * 
 */
//input form stellarium
char inputChar[20];
//init RAHH,RAMM,RASS to visualize the star pointer;
char RAHH[3]="12",RAMM[3]="00",RASS[3]="00";
//init DECSign,DECDeg,DECMM,DECSS to visualize the star pointer;
char DECSign[2]="-",DECDeg[3]="12",DECMM[3]="00",DECSS[3]="00";
//convert RA/DEC into degree
float RADegree, DECDegree;
char SIGNtel;
float DEC_Cal;
float HA_Cal;
float Ra_Cal;
char txRA[10];                     
char txDEC[11];
float RAHH_Cal;
float RAMM_Cal;
float RASS_Cal;
float DECDeg_Cal;
float DECMM_Cal;
float DECSS_Cal;
float DEC_Cal1;
float Ra_Cal1;

/**
 * Include library for OLED by use I2C  
 * 
 */
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"
#define I2C_ADDRESS 0x3C
SSD1306AsciiAvrI2c oled;

/**
 * Define the global variable for the Communicate between GPS module and Arduino  
 * 
 */
#include "SPI.h"        // necessary library
#define DATAOUT 11      //MOSI
#define DATAIN  12      //MISO
#define SPICLOCK  13    //sck
#define SLAVESELECT 10  //ss
byte clr;
byte spi_output_data;
char gpsStream;

// SPI transmiss function
char spi_transfer(volatile char data)
{
  SPDR = data;                    // Start the transmission
  while (!(SPSR & (1<<SPIF)))     // Wait the end of the transmission
  {
  };
  return SPDR;                    // return the received byte
}

// read out the data through SPI 
byte read()
{
  int data;
  digitalWrite(SLAVESELECT,LOW);
  data = spi_transfer(0x00);        //get data byte
  digitalWrite(SLAVESELECT,HIGH);   //release chip, signal end transfer
  return data;
}


/**
 * GPS data parsing setting  
 * 
 */
#include <TinyGPS++.h>
TinyGPSPlus gps;
int Year = 2020;                            // Date (Year)
int Month = 10;                             // Date (Month)
int Day  = 10;                              // Date (Day) 
int timeHH=12;                              // Time (Hours)
int timeMM=00;                              // Time (Minutes)
int timeSS=0;                               // Time (Seconds)
float Latitude;                             // Latitude (signed degrees)
float Longitude;                            // Longitude (signed degrees)
#define pi 3.14159265
#define rad2deg(X) ((X)/pi*180)             //Rad -->> degree
#define deg2rad(X) ((X)/180*pi)             //Degree-->>rad
float Timenow;
float Alt = 0.0, Az = 0.0;                  // Target star Altitude and Azimuth
float ra;                                 
float dec;

/**
 * Servo setting  
 * 
 */
#include <Servo.h>               // used to control the servos
Servo servo_alt, servo_az;
float object_alt = 0.0;         // this is north and horizontal
float object_az  = 0.0;
float position_alt = object_alt;
float position_az = object_az;
int slew_speed = 10;             // this is a delay in ms
int received_az = position_az, received_alt = position_alt;

/**
 * This function is to update the data from GPS module  
 * 
 */
void update_gps(){
  spi_output_data = read();
  delay(100); //pause for readability
  if (gps.encode((char)spi_output_data)){
    if (gps.location.isValid()){
      Latitude= gps.location.lat();
      Longitude= gps.location.lng();
      }else{
        Latitude=48.5216; 
        Longitude=9.0576;
        }
    oled.setCursor(0, 2);
    oled.print(Longitude);
    if (gps.date.isValid()){   
      Month= gps.date.month();
      Day= gps.date.day();
      Year= gps.date.year();
      oled.setCursor(0,5);
      oled.print(Year);
        }
    if (gps.time.isValid()){
       timeHH= gps.time.hour();
       timeMM= gps.time.minute();
       timeSS= gps.time.second();
       Timenow = ((float)timeHH + (((((float)timeSS + 15)/60.0) + (float)timeMM) /60.0)); // gps time +15 = UT, UT to decimal hours
       oled.setCursor(0,6);
       oled.print(timeMM);
        }   
      }
}

/**
 * Convert string that we get from stellarium into float  
 * 
 */
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

/**
 * this function is use to made the calculation of the coordinate  
 * 
 */
void RADEC_2_AZALT(){
   
  //base on 2011 - Practical Astronomy with your Calculator or Spreadsheet_4thEd - DUFFETT-SMITH
  //julian date at 0h 
  float Today =  float(Day); //longitude in the west need to minus 1, today like 2009 June  (i.e. 6 pm on 19 June is 19.75).
  if(Month == 1 || Month == 2){
    Month = Month + 12;
    Year = Year - 1;
  }
  int a = floor(Year / 100);                   // floor to get trunc
  int b = 2 - a + floor(a / 4);
  float jd = floor(365.25 * (Year + 4716)) + floor(30.6001 * (Month + 1)) + Today + b - 1524.5;
  
  //UT_TO_GST
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
  
  // GST_TO_LST
  //convers Greenwich time to Local sidereal time
  float geolon = (Longitude/15);  //Convert the geographical longitude in degrees to its equivalent in hours by dividing by 15.
  float lst;
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
  
 //Calculation hour angle
 //Hour Angle of star = Local Sidereal Time- Right Ascension of star
 float Ha = lst - (RADegree/15);
 Ha = Ha*15; // Multiply by 15 to convert Ha to degrees
 if (Ha < 0){
 Ha = 360 + Ha;
 }
 
 //ALT AZ calculations
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
 
 // sin Dec= sinAlt*sinLat+cosAlt*cosLat*cosAZ
 // cosHa=(sinAlt-sinLatsinDec)/cosLatcosDec
 DEC_Cal = rad2deg(asin(sin(deg2rad(Alt))*sin(deg2rad(Latitude))+cos(deg2rad(Alt))*cos(deg2rad(Latitude))*cos(deg2rad(Az))));
 HA_Cal = rad2deg(acos((sin(deg2rad(Alt)) - sin(deg2rad(Latitude))*sin(deg2rad(DEC_Cal)))/(cos(deg2rad(Latitude))*cos(deg2rad(DEC_Cal)))));
 
 if (rad2deg(sin(deg2rad(Az))) > 0){
    HA_Cal = 360 - HA_Cal;
  }
 Ra_Cal = ((lst*15) - HA_Cal);
 if (Ra_Cal > 180){
    Ra_Cal = Ra_Cal -360;
    if (0 <cos(HA_Cal) && cos(HA_Cal)< 0.5){
      Ra_Cal = Ra_Cal + 360;
    }
  }
  oled.setCursor(0, 6);
  oled.print("DEC:");
  oled.print(DEC_Cal); 
}

/**
 *  Drive the Servo to point to a new target
 * 
 */
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

 
/**
 * This function is use to get the position to feedack to the Stellarium after coordinate   
 * convert
 * 
 */
void Get_coordinate(){
  if (DEC_Cal < 0){
   DEC_Cal1 = abs(DEC_Cal);
  }else if (DEC_Cal >= 0){
   DEC_Cal1 = DEC_Cal;
  }
  DECDeg_Cal = DEC_Cal1;
  DECMM_Cal = (DEC_Cal1 - int(DECDeg_Cal)) * 60;
  DECSS_Cal = (DECMM_Cal-int(DECMM_Cal))*60;
  //(DEC_Cal < 0) ? sign = 45 : sign = 43;
  //oled.setCursor(0, 7);
  //oled.print("DEC:");
  //oled.print(int(DECDeg_Cal));
  //oled.print('/');
  //oled.print((DECMM_Cal));
  //oled.print('/');
  //oled.print(DECSS_Cal);
  // RA calculation
  if (Ra_Cal < 0){
   Ra_Cal1 = Ra_Cal + 360;
  }else if (DEC_Cal >= 0){
   Ra_Cal1 = Ra_Cal;
  }
  RAHH_Cal = int(Ra_Cal1/15);
  RAMM_Cal = ((Ra_Cal1-int(RAHH_Cal*15))*60)/15;
  RASS_Cal = ((RAMM_Cal - int(RAMM_Cal))*60)/15;
  
  (DEC_Cal < 0) ? SIGNtel = 45: SIGNtel = 43;
  sprintf(txRA, "%02d:%02d:%02d#", int(RAHH_Cal), int(RAMM_Cal), int(RASS_Cal));
  sprintf(txDEC, "%c%02d%c%02d:%02d#", SIGNtel, int(DECDeg_Cal), 223, int(DECMM_Cal), int(DECSS_Cal));
}


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
  //sample on leading edge of clk,system clock/4 rate (fastest)
  SPCR = (1<<SPE)|(1<<MSTR)| (1<<SPR1) | (1<<SPR0);
  clr=SPSR;
  clr=SPDR; 
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
  oled.setFont(System5x7);
  oled.clear();
  servo_az.attach(8); // azimut servo is on pin D8
  servo_alt.attach(7); // altitude servo is on pin D7
  // set init position (north/horizontal)
}

void loop() {
  
  update_gps();
  getRADEC_to_Degree();
  RADEC_2_AZALT();
 
  if(Serial.available()){
    int i= 0;
    //read out the command from Stellarium
    inputChar[i++]= Serial.read();
    oled.setCursor(0, 1);
    oled.print(inputChar);
    delay(5);
    //waiting 
    while((inputChar[i++]=Serial.read()) != '#'){
      delay(5);
    }
    ////ask telescope present RA #:GR#
    if(inputChar[1]==':' && inputChar[2]=='G' && inputChar[3]=='R' && inputChar[4]=='#'){
      Get_coordinate();
      Serial.print(txRA);
    }
    //ask telescope present DEC #:GD#
    if(inputChar[1]==':' && inputChar[2]=='G' && inputChar[3]=='D' && inputChar[4]=='#'){
     Get_coordinate();
     Serial.print(txDEC);
    }
    // get ":Q#"
    if(inputChar[1]==':' && inputChar[2]=='Q' && inputChar[3]=='#'){
      //Do nothing with Stellarium
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
      
    }
    //:MS# Slew to Target Object
    if(inputChar[0]==':' && inputChar[1]=='M' && inputChar[2]=='S' && inputChar[3]=='#'){    
      // RETURNS 0 means OK
      Serial.print("0");  
      // position pointing 
      set_azimut((int)object_az);
      set_altitude((int)object_alt);
    } 
  }
}
