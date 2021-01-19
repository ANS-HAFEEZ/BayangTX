/////////////////////////////////////////////////////// OLED ////////////////////////////////////////////////////////////////////




#define OLED_CLK   9
#define OLED_MOSI  12
#define OLED_RESET 11
#define OLED_DC    10

//////////////////////////////////////////////////////// Sticks and Buttons ///////////////////////////////////////////////////////
#define T_PIN  A0
#define R_PIN  A1
#define P_PIN  A2
#define Y_PIN  A3

#define B1_PIN  6
#define B2_PIN  5
#define B3_PIN  A6
#define B4_PIN  A7

#define S1_PIN  7
#define S2_PIN  8

///////////////////////////////////////////////////////////// nRF24l0  ///////////////////////////////////////////////////////////////

//SPI Comm.pins with nRF24L01
#define MOSI_pin  2  // MOSI - D3
#define SCK_pin   3  // SCK  - D4
#define CE_pin    4  // CE   - D5
#define MISO_pin  A4 // MISO - A0
#define CS_pin    A5 // CS   - A1

#define ledPin    13 // LED  - D13
// SPI outputs
#define MOSI_on PORTD |= _BV(2)  // PD3
#define MOSI_off PORTD &= ~_BV(2)// PD3
#define SCK_on PORTD |= _BV(3)   // PD4
#define SCK_off PORTD &= ~_BV(3) // PD4
#define CE_on PORTD |= _BV(4)    // PD5
#define CE_off PORTD &= ~_BV(4)  // PD5
#define CS_on PORTC |= _BV(5)    // PC1
#define CS_off PORTC &= ~_BV(5)  // PC1
// SPI input
#define  MISO_on (PINC & _BV(4)) // PC0
