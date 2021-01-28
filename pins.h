/////////////////////////////////////////////////////// OLED ////////////////////////////////////////////////////////////////////

#define OLED_CLK   9
#define OLED_DC    10
#define OLED_RESET 11
#define OLED_MOSI  12

//////////////////////////////////////////////////////// Sticks and Buttons ///////////////////////////////////////////////////////
#define T_PIN  A1
#define R_PIN  A2
#define P_PIN  A3
#define Y_PIN  A0

#define ReadT analogRead(T_PIN)
#define ReadR analogRead(R_PIN)
#define ReadP analogRead(P_PIN)
#define ReadY analogRead(Y_PIN)

#define B1_PIN  6
#define B2_PIN  5
#define B3_PIN  A6
#define B4_PIN  A7

#define S1_PIN  7
#define S2_PIN  8

///////////////////////////////////////////////////////////// CC2500  ///////////////////////////////////////////////////////////////

#define MO_pin 5    //D5
#define MI_pin 6    //D6
#define SCLK_pin 4  //D4
#define CS 2        //D2

#define  SCK_on PORTD |= 0x10       //D4
#define  SCK_off PORTD &= 0xEF      //D4

#define  MO_on PORTD |= 0x20        //D5 
#define  MO_off PORTD &= 0xDF       //D5
//
#define  MI_1 (PIND & 0x40) == 0x40 //D6 input
#define  MI_0 (PIND & 0x40) == 0x00 //D6
//
#define  CS_on PORTD |= 0x04        //D2
#define  CS_off PORTD &= 0xFB       //D2 
