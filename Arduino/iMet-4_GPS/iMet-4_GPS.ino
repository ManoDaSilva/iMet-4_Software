/*
  iMET-4. Script to use the board as a GPS "Mouse".

  CREDITS:
  -Amit Ate from Uravu Labs for the MS5607 library: https://github.com/UravuLabs/MS5607


*/

//LIBRARIES
#include "Pins.h"


HardwareSerial Serial1(USART1_RX, USART1_TX);
//HardwareSerial Serial2(USART2_RX, USART2_TX); //Not understood yet, but the 2nd (GPS) serial should not be initialized here



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
  Serial1.begin(9600); //Start debug interface (to edge card connector)
  Serial1.println("I'm alive!");

  Serial2.begin(9600); //Start GPS interface




}

void loop() {
  serialForward();
}

void powerOff() {
  digitalWrite(P_ON, LOW);
  delay(2000);
}



void serialForward() {
  if (Serial2.available()) {      // If anything comes in Serial (USB),
    Serial1.write(Serial2.read());   // read it and send it out Serial1 (pins 0 & 1)
  }

  if (Serial1.available()) {     // If anything comes in Serial1 (pins 0 & 1)
    Serial2.write(Serial1.read());   // read it and send it out Serial (USB)
  }
}
