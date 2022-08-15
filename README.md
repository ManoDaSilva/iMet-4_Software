# iMet-4_Software
 Some (WIP) software for iMet-4 radiosondes

## Arduino scripts
To compile those and use them with the Arduino IDE, you'll need the following:
* STM32 MCU Base Boards support package (you can find it in Boards Manager)
* STM32F373 boards support (not yet included by default). See Arduino/AddToBoards.txt 

Some libraries might be required:
* TinyGPS++
* RadioLib 

This is still heavily WIP, a radio transmitter lib must be written (perhaps deriving work from the CC1101 lib included?)