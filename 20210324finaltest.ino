#include <NeoSWSerial.h>
/*-------------------------------
---------SPI setting-------------
================================*/
#include "SPI.h" // necessary library
#define DATAOUT 11//MOSI
#define DATAIN  12//MISO
#define SPICLOCK  13//sck
#define SLAVESELECT 10//ss
#define HIGH 0
#define LOW 1
byte clr;
//data buffer
double lst;
byte output_data;
char signDEC; // sign of DEC in the inverse calculations(that is AzAlt2RaDec)
char spi_transfer(volatile char data)
{
  SPDR = data;                    // Start the transmission
  while (!(SPSR & (1<<SPIF)))     // Wait the end of the transmission
  {
  };
  return SPDR;                    // return the received byte
}

/*-------------------------------
---------OLED setting-------------
================================*/
#define I2C_ADDRESS 0x3C
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"
#define I2C_ADDRESS 0x3C
SSD1306AsciiAvrI2c oled;
/*-------------------------------
---------GPS setting-------------
================================*/
#include <TinyGPS++.h>
TinyGPSPlus gps;
int Year = 2020;                            // Date (Year)
int Month = 10;                             // Date (Month)
int Day  = 10;                              // Date (Day) 
int timeHH=12;                              // Time (Hours)
int timeMM=00;                              // Time (Minutes)
int timeSS=0;                               // Time (Seconds)
float Latitude=48.5216;                     // Latitude (signed degrees)
float Longitude=9.0576;                     // Longitude (signed degrees)
int32_t  no_satellites;                     // Number of satellites
#define pi 3.14159265
#define rad2deg(X) ((X)/pi*180)
#define deg2rad(X) ((X)/180*pi)
double Timenow;
char txRA[10];// Used in reply to stellarium
char txDEC[11];//Used in reply to stellarium
/*-------------------------------
---------COODINATE CONVERT-------------
================================*/
double juliandate;
double gstresult;
double lstresult;
//hour angle 时角
float HA;
//高度，方位  Altitude and Azimuth 
float Alt = 0.0, Az = 0.0; // Target star Altitude and Azimuth
float Alt_position = 0, Az_position = 0; // Servo position of Target star Altitude and Azimuth
double ra; //These variables are used in the calculations, double/float/int/byte depending on the type of number needed.
double dec;
/*-------------------------------
---------COMMUNICATION-------------
================================*/
//input form stellarium
char inputChar[20];
//init RAHH,RAMM,RASS to visualize the star pointer;
char RAHH[3]="12",RAMM[3]="00",RASS[3]="00";
//init DECSign,DECDeg,DECMM,DECSS to visualize the star pointer;
char DECSign[2]="-",DECDeg[3]="12",DECMM[3]="00",DECSS[3]="00";
String RAtargetString, DECtargetString;
float RADegree, DECDegree;

#include <Servo.h>    // used to control the servos
Servo servo_alt, servo_az;
double object_alt = 0.0; // this is north and horizontal
double object_az  = 0.0;
double position_alt = object_alt;
double position_az = object_az;
int slew_speed = 10; // this is a delay in ms
int received_az = position_az, received_alt = position_alt;
float object_RA  = 37.9632083;  // in decimal degrees
float object_DEC = 89.2643056;  // in decimal degrees
float position_RA  = object_RA;  // in decimal degrees
float position_DEC = object_DEC;  // in decimal degrees
void setup() {
  Serial.begin(9600);     // Main Baud Rate
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
  //set_azimut((int)position_az);
  //set_altitude((int)position_alt);
}
byte read()
{
  int data;
  digitalWrite(SLAVESELECT,LOW);
  data = spi_transfer(0x00); //get data byte
  digitalWrite(SLAVESELECT,HIGH); //release chip, signal end transfer
  return data;
}
void loop() { 
   while(Serial.available()){
    // communicate with Stellarium 
       todoActions();
       //update_gps();
       //getRADEC_in_Degree();
       //juliandate = julian();
       //gstresult = UT_to_GST();
       //lstresult = GST_to_LST();
       //HA = hourangle();
       //azalt();
       oled.setCursor(0, 0); 
   }
}
void todoActions(){
  int i=0;
    inputChar[i++] = Serial.read();
    delay(5);
    while((inputChar[i++] = Serial.read()) != '#'){
      delay(5);
    }
    inputChar[i]='\0';
    oled.setCursor(0, 9); 
     oled.print(inputChar); 
    //inputString=inputChar;
      //ask telescope RA
  //ask telescope present RA 命令#:GR# stellarium询问现在望远镜指向的RA值
  if(inputChar[1]==':' && inputChar[2]=='G' && inputChar[3]=='R' && inputChar[4]=='#'){
    answerTelRA();//answer telescope RA
  }
  //ask telescope present DEC 命令#:GD# stellarium询问现在望远镜指向的DEC值
  if(inputChar[1]==':' && inputChar[2]=='G' && inputChar[3]=='D' && inputChar[4]=='#'){
    answerTelDEC();//answer telescope DEC
  }
  if(inputChar[1]==':' && inputChar[2]=='Q' && inputChar[3]=='#'){
  // quite stop servo motor
  }
  //arduino get targetRA when user type ctrl+1 #:Q#:Sr 10:10:10#  stellarium发送目标RA值命令 
  if(inputChar[0]==':' && inputChar[1]=='S' && inputChar[2]=='r'){
    getTargetRA();  
  }
  //arduino get targetDEC when user    type ctrl+1 #:Sd +76:24:55# stellarium发送目标DEC值命令
  if(inputChar[0]==':' && inputChar[1]=='S' && inputChar[2]=='d'){
    getTargetDEC();   
  }
  if(inputChar[0]==':' && inputChar[1]=='M' && inputChar[2]=='S' && inputChar[3]=='#'){    //:MS# Slew to Target Object
    //RETURNS 0 means OK
    Serial.print("0");
    
    // set the new coordinates as current coordinates
  }  
}
  
void answerTelRA(){
  Serial.print(RAHH);
  Serial.print(":");
  Serial.print(RAMM);
  Serial.print(":");
  Serial.print(RASS);
  Serial.print("#");
}

void answerTelDEC(){
  Serial.print(DECSign);
  Serial.print(DECDeg);
  Serial.print((char)223);
  Serial.print(DECMM);
  Serial.print(":");
  Serial.print(DECSS);
  Serial.print("#");
}

void getTargetRA(){
  Serial.print("1");
  RAHH[0]=inputChar[3];
  RAHH[1]=inputChar[4];
  RAMM[0]=inputChar[6];
  RAMM[1]=inputChar[7];
  RASS[0]=inputChar[9];
  RASS[1]=inputChar[10];
}

void getTargetDEC(){
  Serial.print("1");
  DECSign[0]=inputChar[3];
  DECDeg[0]=inputChar[4];
  DECDeg[1]=inputChar[5];
  DECMM[0]=inputChar[7];
  DECMM[1]=inputChar[8];
  DECSS[0]=inputChar[10];
  DECSS[1]=inputChar[11];
}
void getRADEC_in_Degree(){
  //RAHH in char, get RA Degree which is hour *15, one hour = 15 degrees
  RADegree = (atof(RAHH) + (atof(RAMM)/60) +(atof(RASS)/3600)) *15;   // degrees in decimal
  //get DEC Degree
  
  DECDegree = (atof(DECDeg) + atof(DECMM)/60 + atof(DECSS)/3600);  // degrees in decimal
  if (DECSign[0] == '-'){
    DECDegree = 0- DECDegree;
  }
}
float julian(){                                       // follow the  book [Practical Astronomy with your Calculator or Spreadsheet_4thEd]
  //julian date at 0h
  double Today =  double(Day);                      //longitude in the west need to minus 1, today like 2009 June  (i.e. 6 pm on 19 June is 19.75).
  if(Month == 1 || Month == 2){
    Month = Month + 12;
    Year = Year - 1;
  }
  int a = floor(Year / 100);                   // floor to get trunc
  int b = 2 - a + floor(a / 4);
  float jd = floor(365.25 * (Year + 4716)) + floor(30.6001 * (Month + 1)) + Today + b - 1524.5;
  return jd;                                                 // get the Julianday i.e. jd = 2455002.25
}
float UT_to_GST(){                                            //converts Univeral time to Greenwich Sideral time 
  double s = juliandate - 2451545.0;
  double t = s / 36525.0;
  double T0 = 6.697374558 + (2400.051336 * t) + (0.000025862 * t * t);
  //Reduce the result to the range 0 to 24 by adding or subtracting multiples of 24.
  double n1 = floor (T0 /24);
         T0 = T0 - (n1 * 24);
  double h1 = (Timenow * 1.002737909);
  //reduce to the range 0 to 24 if necessary by subtracting or adding 24.
  double n2 = floor((T0 + h1)/ 24.0);
  double gst = (T0 + h1)- (n2 * 24.0);
  return gst;                                               // result with Decimal
}
float GST_to_LST(){   //convers Greenwich time to Local sidereal time
  
  double geolon = (Longitude/15);  //Convert the geographical longitude in degrees to its equivalent in hours by dividing by 15.
  
  if ((Longitude * -1) > 0){
        gstresult = gstresult -geolon;
  }
  else {
    gstresult = gstresult + geolon;
  }
  if (gstresult > 24){
    lst = gstresult - 24;
  }
  else if ((gstresult * -1)>0){
    lst = gstresult +24;
  }
  else{
    lst = gstresult;
  }
  return lst;               // result with Decimal in degrees
}
float hourangle(){ //Hour Angle of star = Local Sidereal Time- Right Ascension of star
 float Ha = lstresult - (RADegree/15);
 Ha = Ha*15; // Multiply by 15 to convert Ha to degrees
 if (Ha < 0){
 Ha = 360 + Ha;
 }
 return Ha ; //hour angle in degrees
}
void azalt() { //This section calculates the Azimuth and the Altitude of the target object.
  oled.setCursor(0, 10);
  oled.print(HA);
  dec = ((DECDegree / 360) * (2 * PI));
 Alt=rad2deg(asin(sin(deg2rad(DECDegree))*sin(deg2rad(Latitude))+cos(deg2rad(DECDegree))*cos(deg2rad(Latitude))*cos(deg2rad(HA))));
 Az=rad2deg(acos((sin(deg2rad(DECDegree)) -(sin(deg2rad(Alt))*sin(deg2rad(Latitude))))/(cos(deg2rad(Alt)) * cos(deg2rad(Latitude)))));
 //if sin(HA) is positive, the angle AZ is 360 - AZ
 if (rad2deg(sin(deg2rad(HA))) > 0) {
 Az= 360 - Az;
 }
 if ((Alt*-1)>0) { //If altitude is below 0 degrees, then the object is below the observer's horizon.
     oled.setCursor(0, 9); 
     oled.print("IS BELOW HORIZON"); 
    }
}
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
  
  int RAHH_Cal = Ra_Cal/15;
  int RAMM_Cal = ((Ra_Cal-(RAHH_Cal*15))*60)/15;
  int RASS_Cal = ((Ra_Cal-(RAHH_Cal*15 + (RAMM_Cal*15)/60))*3600)/15;
  int DECDeg_Cal = int(DEC_Cal);
  int DECMM_Cal = int((DEC_Cal-DECDeg_Cal)*60);
  int DECSS_Cal = (DEC_Cal-DECDeg_Cal-DECMM_Cal/60)*3600;
  (DEC_Cal < 0) ? signDEC = 45 : signDEC = 43;
  sprintf(txRA, "%02d:%02d:%02d#", RAHH_Cal,RAMM_Cal,RASS_Cal);
  sprintf(txDEC, "%c%02d%c%02d:%02d#", signDEC, DECDeg_Cal, 223, DECMM_Cal, DECSS_Cal);
  
}
