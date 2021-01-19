#include <util/atomic.h>
#include <EEPROM.h>
#include "iface_nrf24l01.h"

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
#define GET_FLAG(ch, mask) (PPM[ch] > PPM_MAX_COMMAND ? mask : 0)
#define GET_FLAG_INV(ch, mask) (PPM[ch] < PPM_MIN_COMMAND ? mask : 0)

// supported protocols
enum {
    PROTO_BAYANG = 5,
    PROTO_BAYANG_SILVERWARE = 16
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
    uint8_t  updated;
    uint32_t lastUpdate;
} telemetry_data;

uint8_t transmitterID[4];
uint8_t current_protocol;
uint8_t packet[32];

static uint16_t PPM[12] = {PPM_MIN,PPM_MIN,PPM_MIN,PPM_MIN,PPM_MID,PPM_MID, PPM_MID,PPM_MID,PPM_MID,PPM_MID,PPM_MID,PPM_MID,};

static int STICKnAUX[6] = {A0, A1, A2, A3, A4, A5};

void setup(){
    Serial.begin(115200);
   
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, LOW); //start LED off
    pinMode(MOSI_pin, OUTPUT);
    pinMode(SCK_pin, OUTPUT);
    pinMode(CS_pin, OUTPUT);
    pinMode(CE_pin, OUTPUT);
    pinMode(MISO_pin, INPUT);

    pinMode(10,INPUT_PULLUP);
    pinMode(9,INPUT_PULLUP);
    
    for (int i = 0; i < 6; i++){
        pinMode(STICKnAUX[i],INPUT_PULLUP);
    }
    set_txid(false);
}

void loop(){
  if(PPM[BIND_AUX] > PPM_MAX_COMMAND) {  
    current_protocol = PROTO_BAYANG;   
    if(PPM[YAW] < PPM_MIN_COMMAND){
      set_txid(true);
    }
    
    EEPROM.update(ee_PROTOCOL_ID, current_protocol);
    while(PPM[THROTTLE] > PPM_SAFE_THROTTLE) {
      delay(100);
      update_ppm();
    }

    NRF24L01_Reset();
    NRF24L01_Initialize();
    Bayang_init();
		Bayang_bind();
  }
  process_Bayang();
	update_ppm();
}

void set_txid(bool renew){
  uint8_t i;
  for(i=0; i<4; i++){
    transmitterID[i] = EEPROM.read(ee_TXID0+i);
  }
  if(renew || (transmitterID[0]==0xFF && transmitterID[1]==0x0FF)) {
    for(i=0; i<4; i++) {
      transmitterID[i] = random() & 0xFF;
      EEPROM.update(ee_TXID0+i, transmitterID[i]); 
    }            
  }
}

void update_ppm(){

}
