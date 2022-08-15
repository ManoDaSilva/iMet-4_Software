/*
  iMET-4 Basic Working Template


  LIBRARY REQUIREMENTS:
  -TinyGPS++
  -RadioLib


  CREDITS:
  -Amit Ate from Uravu Labs for the MS5607 library: https://github.com/UravuLabs/MS5607


*/

//LIBRARIES
#include <TinyGPS++.h>
#include "MS5607_iMet4.h"
#include <SPI.h>
#include "CC1101_RF.h"
#include "Pins.h"


HardwareSerial Serial1(USART1_RX, USART1_TX);
//HardwareSerial Serial2(USART2_RX, USART2_TX); //Not understood yet, but the 2nd (GPS) serial should not be initialized here

MS5607 P_Sens;
TinyGPSPlus gps;
CC1101 radio;


//GLOBAL VARIABLES DEFINITIONS
float P_val, T_val, H_val;

void setup() {
  //GPIO PIN DEFINITIONS
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);
  pinMode(P_ON, OUTPUT);

  //Turn power pin ON
  digitalWrite(P_ON, HIGH);

  //Turn on LED1 to show it's alive
  digitalWrite(LED1, HIGH);

  //Initialize Serial Interfaces
  Serial1.begin(115200); //Start debug interface (to edge card connector)
  Serial1.println("I'm alive!");

  Serial2.begin(9600); //Start GPS interface

  MS5607_init(); //Start Pressure Sensor

  SPI.setMISO(SPI1_MISO);
  SPI.setMOSI(SPI1_MOSI);
  SPI.setSCLK(SPI1_SCK);
  SPI.setSSEL(SPI1_SS);

  SPI.begin(); // mandatory. CC1101_RF does not start SPI automatically
  radio.begin(433.2e6); // Freq=433.2Mhz





}

void loop() {
  //MS5607_readVals();
  //GPS_readVals();
  //printVals();

}

void powerOff() {
  digitalWrite(P_ON, LOW);
  delay(2000);
}


void MS5607_init() {
  if (!P_Sens.begin()) {
    Serial1.println("Error in Communicating with sensor, check your connections!");
  } else {
    Serial1.println("MS5607 initialization successful!");
  }
}

void MS5607_readVals() {
  if (P_Sens.readDigitalValue()) {
    T_val = P_Sens.getTemperature();
    P_val = P_Sens.getPressure();
    H_val = P_Sens.getAltitude();
  } else {
    Serial.println("Error in reading digital value in sensor!");
  }
}

void GPS_readVals() {
  while (Serial2.available() > 0) {
    if (gps.encode(Serial2.read()))
    {
    }
  }
}

void printVals() {
  Serial1.print("Temperature :  ");
  Serial1.print(T_val);
  Serial1.println(" C");
  Serial1.print("Pressure    :  ");
  Serial1.print(P_val);
  Serial1.println(" mBar");
  Serial1.print("Altitude    :  ");
  Serial1.print(H_val);
  Serial1.println(" meter");
  Serial1.print(F("Location: "));
  if (gps.location.isValid())
  {
    Serial1.print(gps.location.lat(), 6);
    Serial1.print(F(","));
    Serial1.print(gps.location.lng(), 6);
  }
  else
  {
    Serial1.print(F("INVALID"));
  }

  Serial1.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial1.print(gps.date.month());
    Serial1.print(F("/"));
    Serial1.print(gps.date.day());
    Serial1.print(F("/"));
    Serial1.print(gps.date.year());
  }
  else
  {
    Serial1.print(F("INVALID"));
  }

  Serial1.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial1.print(F("0"));
    Serial1.print(gps.time.hour());
    Serial1.print(F(":"));
    if (gps.time.minute() < 10) Serial1.print(F("0"));
    Serial1.print(gps.time.minute());
    Serial1.print(F(":"));
    if (gps.time.second() < 10) Serial1.print(F("0"));
    Serial1.print(gps.time.second());
    Serial1.print(F("."));
    if (gps.time.centisecond() < 10) Serial1.print(F("0"));
    Serial1.print(gps.time.centisecond());
  }
  else
  {
    Serial1.print(F("INVALID"));
  }

  Serial1.println();
}
