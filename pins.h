/////////////////////////////////////////////////////// OLED ////////////////////////////////////////////////////////////////////
#define OLED_MOSI  12
#define OLED_CLK   6
#define OLED_DC    11
#define OLED_CS    0
#define OLED_RESET 13



//////////////////////////////////////////////////////// Sticks and Buttons ///////////////////////////////////////////////////////
#define T_PIN  A2
#define R_PIN  A0
#define P_PIN  A1
#define Y_PIN  A3

#define B1_PIN  A4
#define B2_PIN  A5
#define B3_PIN  A6
#define B4_PIN  A7

#define S1_PIN  2
#define S2_PIN  3

///////////////////////////////////////////////////////////// nRF24l0  ///////////////////////////////////////////////////////////////

//SPI Comm.pins with nRF24L01
#define MOSI_pin  2  // MOSI - D2
#define SCK_pin   3  // SCK  - D3
#define CE_pin    4  // CE   - D4
#define CS_pin    5  // CS   - D5
#define MISO_pin  6  // MISO - D6

#define ledPin    13 // LED  - D13
// SPI outputs
#define MOSI_on  PORTD |=  _BV(2) // PD2
#define MOSI_off PORTD &= ~_BV(2) // PD2
#define SCK_on   PORTD |=  _BV(3) // PD3
#define SCK_off  PORTD &= ~_BV(3) // PD3
#define CE_on    PORTD |=  _BV(4) // PD4
#define CE_off   PORTD &= ~_BV(4) // PD4
#define CS_on    PORTD |=  _BV(5) // PD5
#define CS_off   PORTD &= ~_BV(5) // PD5
// SPI input
#define  MISO_on (PIND & _BV(6))  // PD6
