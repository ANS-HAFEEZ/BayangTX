#include <util/atomic.h>
#include <EEPROM.h>
#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "RunningAverage.h"
#include "pins.h"
#include "iface_nrf24l01.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, 0);

RunningAverage TRA(3);
RunningAverage RRA(3);
RunningAverage PRA(3);
RunningAverage YRA(3);
static bool bIsCalib       = false;
static bool bShowSticks    = false;

static float TOffSet = 0;
static float ROffSet = 0;
static float POffSet = 0;
static float YOffSet = 0;
static int stick=0;
///////////////////////////////////////////////////////////////////////////////////////

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

static uint16_t PPM[12] = {PPM_MIN,PPM_MIN,PPM_MIN,PPM_MIN,PPM_MID,PPM_MID,
                           PPM_MID,PPM_MID,PPM_MID,PPM_MID,PPM_MID,PPM_MID,};

static void ReadFlash(void);
static void WriteFlash(void);
static void ShowSticks(void);
static void CalibSticks(void);

void setup(){
    Serial.begin(115200);
   
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, LOW); //start LED off
    pinMode(MOSI_pin, OUTPUT);
    pinMode(SCK_pin, OUTPUT);
    pinMode(CS_pin, OUTPUT);
    pinMode(CE_pin, OUTPUT);
    pinMode(MISO_pin, INPUT);
    
    set_txid(false);
    
    if(!display.begin()) {
      Serial.println(F("SSD1306 allocation failed"));
      for(;;); // Don't proceed, loop forever
    }
  pinMode(T_PIN,INPUT_PULLUP);
  pinMode(R_PIN,INPUT_PULLUP);
  pinMode(P_PIN,INPUT_PULLUP);
  pinMode(Y_PIN,INPUT_PULLUP);

  pinMode(S1_PIN,INPUT_PULLUP);
  pinMode(S2_PIN,INPUT_PULLUP);

  pinMode(B1_PIN,INPUT_PULLUP);
  pinMode(B2_PIN,INPUT_PULLUP);
  pinMode(B3_PIN,INPUT_PULLUP);
  pinMode(B4_PIN,INPUT_PULLUP);
 
  TRA.clear(); // explicitly start clean
  RRA.clear(); // explicitly start clean
  PRA.clear(); // explicitly start clean
  YRA.clear(); // explicitly start clean
  
  for (int i = 0; i < 10; i++){
    TRA.addValue(0);
    RRA.addValue(0);
    PRA.addValue(0);
    YRA.addValue(0);
  }                
  
  display.clearDisplay();
  display.setTextColor(WHITE);
  if((digitalRead(S2_PIN) ? 1200 : 1800) < 1500){
    bShowSticks = true;
    display.setTextSize(2);
  }else{
    display.setTextSize(9);
    display.setCursor(12,0);
    display.print(F("TX"));
    display.display();
  }
  ReadFlash();
}
static bool reset=true;
void loop(){
  if(reset || PPM[BIND_AUX] > PPM_MAX_COMMAND) {
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
void selectProtocol()
{
    uint8_t count = 10;

    current_protocol = PROTO_BAYANG;   
    if(PPM[YAW] < PPM_MIN_COMMAND)   // Rudder left
        set_txid(true);                      // Renew Transmitter ID
    
     // update eeprom 
    EEPROM.update(ee_PROTOCOL_ID, current_protocol);
    // wait for safe throttle
    while(PPM[THROTTLE] > PPM_SAFE_THROTTLE) {
        delay(100);
        update_ppm();
    }
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
//  Serial.print("S1:");    Serial.print(digitalRead(S1_PIN));
//  Serial.print("\tS2:");  Serial.print(digitalRead(S2_PIN));
//  Serial.print("\tB1:");  Serial.print(digitalRead(B1_PIN));
//  Serial.print("\tB2:");  Serial.print(digitalRead(B2_PIN));
//  Serial.print("\tB3:");  Serial.print(digitalRead(B3_PIN));
//  Serial.print("\tB4:");  Serial.println(digitalRead(B4_PIN));

  TRA.addValue(constrain(map(analogRead(T_PIN),0,600,2000,1000),1050,1900));            
  RRA.addValue(constrain(map(analogRead(R_PIN),0,700,1000,2000),1050,1900));           
  PRA.addValue(constrain(map(analogRead(P_PIN),0,760,1000,2000),1050,1900));            
  YRA.addValue(constrain(map(analogRead(Y_PIN),0,750,1000,2000),1050,1900));  

  PPM[THROTTLE] = TRA.getFastAverage() + TOffSet;
  PPM[ROLL]     = RRA.getFastAverage() + ROffSet;
  PPM[PITCH]    = PRA.getFastAverage() + POffSet;
  PPM[YAW]      = YRA.getFastAverage() + YOffSet;
//
  PPM[FLIP_AUX]    = digitalRead(S1_PIN) ? 1200 : 1800;


//  Serial.print("T:");      Serial.print(PPM[THROTTLE]);
//  Serial.print("\tR:");    Serial.print(PPM[ROLL]);
//  Serial.print("\tP:");    Serial.print(PPM[PITCH]);
//  Serial.print("\tY:");    Serial.print(PPM[YAW]);
//  Serial.print("\tAUX:");  Serial.println(PPM[FLIP_AUX]);

//
//  if(bShowSticks && !bIsCalib){
//    ShowSticks();
//  }
//  else if(bIsCalib){
//    CalibSticks();
//  }
//  else{
//    if(PPM[ROLL]  > 1485 && PPM[ROLL]  < 1515) PPM[ROLL]   = 1500;
//    if(PPM[PITCH] > 1485 && PPM[PITCH] < 1515) PPM[PITCH]  = 1500;
//    if(PPM[YAW]   > 1485 && PPM[YAW]   < 1515) PPM[YAW]    = 1500;
//  }
//  PPM[THROTTLE] = 1000;
}


static void ReadFlash(){
  EEPROM.get(0, TOffSet);
  EEPROM.get(4, ROffSet);
  EEPROM.get(8, POffSet);
  EEPROM.get(12, YOffSet);

  if(abs(TOffSet)> 0 && abs(ROffSet) > 0 && abs(POffSet) > 0 && abs(YOffSet) > 0){}
  else{
    TOffSet = 0;
    ROffSet = 0;
    POffSet = 0;
    YOffSet = 0;
  }
}

static void WriteFlash(){
  EEPROM.put(0, TOffSet);
  EEPROM.put(4, ROffSet);
  EEPROM.put(8, POffSet);
  EEPROM.put(12, YOffSet);
}

static void ShowSticks(){
  if((digitalRead(S1_PIN) ? 1200 : 1800) > 1500){
    display.clearDisplay();
    display.setCursor(0,0);  display.print(F("T: "));
    display.setCursor(0,15); display.print(F("R: "));
    display.setCursor(0,30); display.print(F("P: "));
    display.setCursor(0,45); display.print(F("Y: "));
    
    display.setCursor(22,0);  display.print(PPM[THROTTLE]);
    display.setCursor(22,15); display.print(PPM[ROLL]);
    display.setCursor(22,30); display.print(PPM[PITCH]);
    display.setCursor(22,45); display.print(PPM[YAW]  );
    display.display();
  }else{
    display.clearDisplay();
    display.setCursor(0,0);  display.print(F("T "));
    display.setCursor(0,15); display.print(F("R "));
    display.setCursor(0,30); display.print(F("P "));
    display.setCursor(0,45); display.print(F("Y "));
    
    display.setCursor(22,0);  display.print(TOffSet);
    display.setCursor(22,15); display.print(ROffSet);
    display.setCursor(22,30); display.print(POffSet);
    display.setCursor(22,45); display.print(YOffSet);
    display.display();
  }

  if((digitalRead(S2_PIN) ? 1200 : 1800) > 1500)
    bIsCalib = true;
}

static void CalibSticks(){
    if((map(analogRead(R_PIN),0,1000,1000,2000))> 1700 && (map(analogRead(Y_PIN),0,1000,1000,2000))> 1700){
      delay(500);
      if(stick++>3) stick=0;
    }
    if(stick==THROTTLE){
      if(analogRead(B1_PIN) <200 ){
        TOffSet++;
        delay(100);
      }
      if(analogRead(B3_PIN) <200 ){
        TOffSet--;
        delay(100);
      }
      display.clearDisplay();
      display.setCursor(0,0);   display.print(F("T: "));
      display.setCursor(22,0);  display.print(PPM[THROTTLE]);
      display.setCursor(0,15);  display.print(F("O: "));
      display.setCursor(22,15); display.print(TOffSet);
      display.display();
    }
    else if(stick==ROLL){
      if(analogRead(B1_PIN) <200 ){
        ROffSet++;
        delay(100);
      }
      if(analogRead(B3_PIN) <200 ){
        ROffSet--;
        delay(100);
      }
      display.clearDisplay();
      display.setCursor(0,0);   display.print(F("R: "));
      display.setCursor(22,0);  display.print(PPM[ROLL]);
      display.setCursor(0,15);  display.print(F("O: "));
      display.setCursor(22,15); display.print(ROffSet);
      display.display();
    }
    else if(stick==PITCH){
      if(analogRead(B1_PIN) <200 ){
        POffSet++;
        delay(100);
      }
      if(analogRead(B3_PIN) <200 ){
        POffSet--;
        delay(100);
      }
      
      display.clearDisplay();
      display.setCursor(0,0);   display.print(F("P: "));
      display.setCursor(22,0);  display.print(PPM[PITCH]);
      display.setCursor(0,15);  display.print(F("O: "));
      display.setCursor(22,15); display.print(POffSet);
      display.display();
    }
    else if(stick==YAW){
      if(analogRead(B1_PIN) <200 ){
        YOffSet++;
        delay(100);
      }
      if(analogRead(B3_PIN) <200 ){
        YOffSet--;
        delay(100);
      }
      display.clearDisplay();
      display.setCursor(0,0);   display.print(F("Y: "));
      display.setCursor(22,0);  display.print(PPM[YAW]);
      display.setCursor(0,15);  display.print(F("O: "));
      display.setCursor(22,15); display.print(YOffSet);
      display.display();
    }

    if((digitalRead(S1_PIN) ? 1200 : 1800) > 1500){
      WriteFlash();
      display.clearDisplay();
      display.setCursor(0,0);   display.print(F("Saved"));
      display.display();
      delay(3000);
      display.clearDisplay();
      display.setCursor(0,0);   display.println(F("Please"));
      display.println(F("Reboot"));
      display.display();
      while(1){}
    }
}
