//
//void chaseLED() {
//  digitalWrite(LED4, LOW);
//  digitalWrite(LED1, HIGH);
//  delay(50);
//  digitalWrite(LED1, LOW);
//  digitalWrite(LED2, HIGH);
//  delay(50);
//  digitalWrite(LED2, LOW);
//  digitalWrite(LED3, HIGH);
//  delay(50);
//  digitalWrite(LED3, LOW);
//  digitalWrite(LED4, HIGH);
//  delay(50);
//  digitalWrite(LED4, LOW);
//  digitalWrite(LED3, HIGH);
//  delay(50);
//  digitalWrite(LED3, LOW);
//  digitalWrite(LED2, HIGH);
//  delay(50);
//  digitalWrite(LED2, LOW);
//  digitalWrite(LED1, HIGH);
//  delay(50);
//}
//
//
//void serialForward() {
//  if (Serial2.available()) {      // If anything comes in Serial (USB),
//    Serial1.write(Serial2.read());   // read it and send it out Serial1 (pins 0 & 1)
//  }
//
//  if (Serial1.available()) {     // If anything comes in Serial1 (pins 0 & 1)
//    Serial2.write(Serial1.read());   // read it and send it out Serial (USB)
//  }
//}
