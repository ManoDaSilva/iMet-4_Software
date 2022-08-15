//GPIO PIN DEFINITIONS
#define LED1 PB14
#define LED2 PB15
#define LED3 PD8
#define LED4 PA8

#define P_ON PB2 //Pin must be set high to keep DC/DC running. 

//INTERFACE PIN DEFINITIONS
#define USART1_RX PA10 //Connected to edge connector pin 2
#define USART1_TX PA9 //Connected to edge connector pin 3
#define USART2_RX PA3 //Connected to uBlox TX
#define USART2_TX PA2 //Connected to uBlox RX

#define I2C1_SDA PB9 //Connected to pressure sensor SDA, but declared in the associated lib.
#define I2C1_SCL PB8 //Connected to pressure sensor SCL, but declared in the associated lib.
