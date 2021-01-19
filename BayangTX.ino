#include <util/atomic.h>
#include <EEPROM.h>
#include "iface_nrf24l01.h"

//SPI Comm.pins with nRF24L01
#define MOSI_pin  3  // MOSI - D3
#define SCK_pin   4  // SCK  - D4
#define CE_pin    5  // CE   - D5
#define MISO_pin  A0 // MISO - A0
#define CS_pin    A1 // CS   - A1

#define ledPin    13 // LED  - D13
// SPI outputs
#define MOSI_on PORTD |= _BV(3)  // PD3
#define MOSI_off PORTD &= ~_BV(3)// PD3
#define SCK_on PORTD |= _BV(4)   // PD4
#define SCK_off PORTD &= ~_BV(4) // PD4
#define CE_on PORTD |= _BV(5)    // PD5
#define CE_off PORTD &= ~_BV(5)  // PD5
#define CS_on PORTC |= _BV(1)    // PC1
#define CS_off PORTC &= ~_BV(1)  // PC1
// SPI input
#define  MISO_on (PINC & _BV(0)) // PC0
#define RF_POWER TX_POWER_5mW 

// PPM stream settings
#define CHANNELS 12 // number of channels in ppm stream, 12 ideally
enum chan_order{
    THROTTLE,
    ROLL,
    PITCH,
    YAW,
    LED_AUX,  // (CH5)  led light, or 3 pos. rate on CX-10, H7, or inverted flight on H101
    FLIP_AUX,  // (CH6)  flip control
    SHOT_AUX,  // (CH7)  still camera (snapshot)
    VID_AUX,  // (CH8)  video camera
    HEADLESS_AUX,  // (CH9)  headless
    RTH_AUX,  // (CH10) calibrate Y (V2x2), pitch trim (H7), RTH (Bayang, H20), 360deg flip mode (H8-3D, H22)
    ESTOP_AUX,  // (CH11) calibrate X (V2x2), roll trim (H7), emergency stop (Bayang, Silverware)
    BIND_AUX,  // (CH12) Reset / Rebind
};

#define PPM_MIN 1000
#define PPM_SAFE_THROTTLE 1100 
#define PPM_MID 1500
#define PPM_MAX 2000
#define PPM_MIN_COMMAND 1300
#define PPM_MAX_COMMAND 1700
#define GET_FLAG(ch, mask) (ppm[ch] > PPM_MAX_COMMAND ? mask : 0)
#define GET_FLAG_INV(ch, mask) (ppm[ch] < PPM_MIN_COMMAND ? mask : 0)

// supported protocols
enum {
    PROTO_V2X2 = 0,     // WLToys V2x2, JXD JD38x, JD39x, JJRC H6C, Yizhan Tarantula X6 ...
    PROTO_CG023,        // EAchine CG023, CG032, 3D X4
    PROTO_CX10_BLUE,    // Cheerson CX-10 blue board, newer red board, CX-10A, CX-10C, Floureon FX-10, CX-Stars (todo: add DM007 variant)
    PROTO_CX10_GREEN,   // Cheerson CX-10 green board
    PROTO_H7,           // EAchine H7, MoonTop M99xx
    PROTO_BAYANG,       // EAchine H8(C) mini, H10, BayangToys X6, X7, X9, JJRC JJ850, Floureon H101
    PROTO_SYMAX5C1,     // Syma X5C-1 (not older X5C), X11, X11C, X12
    PROTO_YD829,        // YD-829, YD-829C, YD-822 ...
    PROTO_H8_3D,        // EAchine H8 mini 3D, JJRC H20, H22
    PROTO_MJX,          // MJX X600 (can be changed to Weilihua WLH08, X800 or H26D)
    PROTO_SYMAXOLD,     // Syma X5C, X2
    PROTO_HISKY,        // HiSky RXs, HFP80, HCP80/100, FBL70/80/90/100, FF120, HMX120, WLToys v933/944/955 ...
    PROTO_KN,           // KN (WLToys variant) V930/931/939/966/977/988
    PROTO_YD717,        // Cheerson CX-10 red (older version)/CX11/CX205/CX30, JXD389/390/391/393, SH6057/6043/6044/6046/6047, FY326Q7, WLToys v252 Pro/v343, XinXun X28/X30/X33/X39/X40
    PROTO_FQ777124,     // FQ777-124 pocket drone
    PROTO_E010,         // EAchine E010, NiHui NH-010, JJRC H36 mini
    PROTO_BAYANG_SILVERWARE, // Bayang for Silverware with frsky telemetry
    PROTO_END
};

// EEPROM locationss
enum{
    ee_PROTOCOL_ID = 0,
    ee_TXID0,
    ee_TXID1,
    ee_TXID2,
    ee_TXID3
};

struct {
    uint16_t volt1;
    uint16_t rssi;
    uint8_t updated;
    uint32_t lastUpdate;
} telemetry_data;

uint8_t transmitterID[4];
uint8_t current_protocol;
static volatile bool ppm_ok = false;
uint8_t packet[32];
static bool reset=true;
volatile uint16_t Servo_data[12];
static uint16_t ppm[12] = {PPM_MIN,PPM_MIN,PPM_MIN,PPM_MIN,PPM_MID,PPM_MID,
                           PPM_MID,PPM_MID,PPM_MID,PPM_MID,PPM_MID,PPM_MID,};


int sticks[4]= {A5,A4,A7,A6};
int auxes[8]= {11,11,11,11,8,11,11,11};



bool fast=false;
void setup()
{
    Serial.begin(115200);
    
    //randomSeed((analogRead(A4) & 0x1F) | (analogRead(A5) << 5));
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, LOW); //start LED off
    pinMode(MOSI_pin, OUTPUT);
    pinMode(SCK_pin, OUTPUT);
    pinMode(CS_pin, OUTPUT);
    pinMode(CE_pin, OUTPUT);
    pinMode(MISO_pin, INPUT);


    pinMode(10,INPUT_PULLUP);
    pinMode(9,INPUT_PULLUP);
    
    for (int i = 0; i < CHANNELS; ++i)
    {
        if (i <4)
        {
            pinMode(sticks[i],INPUT_PULLUP);
        }
        else
        {
            pinMode(auxes[i-4],INPUT_PULLUP);
        }
        
    }

    // PPM ISR setup
    //attachInterrupt(digitalPinToInterrupt(PPM_pin), ISR_ppm, CHANGE);
    TCCR1A = 0;  //reset timer1
    TCCR1B = 0;
    TCCR1B |= (1 << CS11);  //set timer1 to increment every 1 us @ 8MHz, 0.5 us @16MHz

    set_txid(false);

    Serial.println("Starting TX");
}

void loop()
{
    if(reset || ppm[BIND_AUX] > PPM_MAX_COMMAND) {
        reset = false;
        selectProtocol();
        NRF24L01_Reset();
        NRF24L01_Initialize();
       	Bayang_init();
		Bayang_bind();
    }
    process_Bayang();
	update_ppm();
}

void set_txid(bool renew)
{
    uint8_t i;
    for(i=0; i<4; i++)
        transmitterID[i] = EEPROM.read(ee_TXID0+i);
    if(renew || (transmitterID[0]==0xFF && transmitterID[1]==0x0FF)) {
        for(i=0; i<4; i++) {
            transmitterID[i] = random() & 0xFF;
            EEPROM.update(ee_TXID0+i, transmitterID[i]); 
        }            
    }
}

void selectProtocol()
{
    ppm_ok = true;
    uint8_t count = 10;

    current_protocol = PROTO_BAYANG;   
    if(ppm[YAW] < PPM_MIN_COMMAND)   // Rudder left
        set_txid(true);                      // Renew Transmitter ID
    
     // update eeprom 
    EEPROM.update(ee_PROTOCOL_ID, current_protocol);
    // wait for safe throttle
    while(ppm[THROTTLE] > PPM_SAFE_THROTTLE) {
        delay(100);
        update_ppm();
    }
}

// update ppm values out of ISR    
void update_ppm()
{
	  ppm[THROTTLE] = map(analogRead(sticks[0]),100,870,2000,1000);

    if(ppm[THROTTLE]<1010)
    {
      ppm[THROTTLE] =1000;  
    }
  
    if(ppm[THROTTLE]>1990)
    {
      ppm[THROTTLE] =1995;  
    }
    
  ppm[ROLL] = constrain(map(analogRead(A7),170,900,1000,2000),1000,2000);            

  if (ppm[ROLL]> 1400 && ppm[ROLL]<1600)
  {
      ppm[ROLL]=1500;
  }
  else if(ppm[ROLL]>1600)
  {
    ppm[ROLL]= 1500+ ppm[ROLL]-1520;
  }
  else
  {
      ppm[ROLL]= 1500 - 1480 + ppm[ROLL];
  }
  
  ppm[PITCH] = constrain(map(analogRead(A4),100,860,2000,1000),1000,2000);            
  if (ppm[PITCH]> 1440 && ppm[PITCH]<1540)
  {
      ppm[PITCH]=1500;
  }
  else if(ppm[PITCH]>1520)
  {
    ppm[PITCH]= 1500+ ppm[PITCH]-1520;
  }
  else
  {
      ppm[PITCH]= 1500 - 1480 + ppm[PITCH];
  }
  
  ppm[YAW] = constrain(map(analogRead(A6),0,1023,2000,1000),1000,2000);            
  
  if (ppm[YAW]> 1440 && ppm[YAW]<1540)
  {
      ppm[YAW]= 1500;
  }
  else if(ppm[YAW]>1540)
  {
    ppm[YAW]= 1500+ ppm[YAW]-1540;
  }
  else
  {
      ppm[YAW]= 1500 - 1440 + ppm[YAW];
  }

//    Serial.print(ppm[THROTTLE]);
//    Serial.print("  ");
  
//  Serial.print(ppm[ROLL]);
//  Serial.print("  ");          
//  
//  Serial.print(ppm[PITCH]);
//  Serial.print("  ");      
////  Serial.print(analogRead(A4));
////  Serial.print("  ");      
//  
//  Serial.print(ppm[YAW]);
//  Serial.print("  ");          
        
            
    ppm[4]=1000;    
    ppm[5]=1000;    
    ppm[6]=1000;    
    ppm[7]=1000;    
    ppm[8]=1000;    
    ppm[9]=1000;    
    ppm[10]=1000;    
    ppm[11]=1000;    
  
    ppm[FLIP_AUX] = digitalRead(8) ? 1000 : 2000;     
//    Serial.print("\n");
}
