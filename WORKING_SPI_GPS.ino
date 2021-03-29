#include "SPI.h" // necessary library
#define I2C_ADDRESS 0x3C
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"
SSD1306AsciiAvrI2c oled;
#define DATAOUT 11//MOSI
#define DATAIN  12//MISO
#define SPICLOCK  13//sck
#define SLAVESELECT 10//ss
#include <TinyGPS++.h>
TinyGPSPlus gps;
int Year = 2020;
#define HIGH 0
#define LOW 1
byte clr;
//data buffer
char buffer [128];
byte output_data;
char *gpsStream;
void fill_buffer()
{
  for (int I=0;I<128;I++)
  {
    buffer[I]=I;
  }
}

char spi_transfer(volatile char data)
{
  SPDR = data;                    // Start the transmission
  while (!(SPSR & (1<<SPIF)))     // Wait the end of the transmission
  {
  };
  return SPDR;                    // return the received byte
}

void setup()
{
  Serial.begin(9600);
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
}

byte read()
{
  int data;
  digitalWrite(SLAVESELECT,LOW);
  data = spi_transfer(0x00); //get data byte
  digitalWrite(SLAVESELECT,HIGH); //release chip, signal end transfer
  return data;
}

void loop()
{
  output_data = read();
  if (gps.encode((char)output_data))
      displayInfo();
  //if ((int)output_data != 255) //sometimes 255 values come in the output
   // oled.print((char)output_data);
  delay(100); //pause for readability
}
void displayInfo()
{
  oled.println(F("Location: ")); 
  if (gps.location.isValid())
  {
    oled.print(gps.location.lat(), 6);
    oled.print(F(","));
    oled.print(gps.location.lng(), 6);
  }
  else
  {
    oled.print(F("INVALID"));
  }

  oled.println(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    oled.print(gps.date.month());
    oled.print(F("/"));
    oled.print(gps.date.day());
    oled.print(F("/"));
    oled.print(gps.date.year());
  }
  else
  {
    oled.print(F("INVALID"));
  }

  oled.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) oled.print(F("0"));
    oled.print(gps.time.hour());
    oled.print(F(":"));
    if (gps.time.minute() < 10) oled.print(F("0"));
    oled.print(gps.time.minute());
    oled.print(F(":"));
    if (gps.time.second() < 10) oled.print(F("0"));
    oled.print(gps.time.second());
    oled.print(F("."));
    if (gps.time.centisecond() < 10) oled.print(F("0"));
    oled.print(gps.time.centisecond());
  }
  else
  {
    oled.print(F("INVALID"));
  }
  oled.println();
}
