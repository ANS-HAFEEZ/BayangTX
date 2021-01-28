#include <util/atomic.h>
#include <EEPROM.h>
#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "RunningAverage.h"
#include "iface_cc2500.h"
#include "pins.h"
//#define DEBUG
//#define DEBUG2
//#define DEBUG3
//#define DEBUG4
#define SPIBB


#define RRangeMax 730
#define PRangeMax 800
#define TRangeMax 600
#define YRangeMax 780


#define SticksMin  1050
#define SticksMax  1900

#define MinMap  1000
#define MaxMap  2000


#define LED_pin A3
#define LED_ON  PORTC |= _BV(3);
#define LED_OFF  PORTC &= ~_BV(3);
#define NOP() __asm__ __volatile__("nop")
//##########Variables########

uint8_t packet[40];
uint8_t counter;
//static byte count=0;
//static byte hop =0; 
//static uint8_t n=0;
static uint32_t state;
static uint16_t PPM_MAX=2000;
static uint16_t PPM_MIN=1000;
static volatile uint16_t PPM[8]={1050,1500,1500,1500,1500,1500,1500,1500};
               
enum {
  FRSKY_BIND        = 0,
  FRSKY_BIND_DONE  = 1000,
  FRSKY_DATA1,
  FRSKY_DATA2,
  FRSKY_DATA3,
  FRSKY_DATA4,
  FRSKY_DATA5
};


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

// PPM stream settings
#define CHANNELS 8 // number of channels in ppm stream, 8 ideally
enum chan_order{
    ROLL,
    PITCH,
    THROTTLE,
    YAW,
    ARM_AUX,
    ACRO_AUX
};

#define PPM_MIN 1000
#define PPM_SAFE_THROTTLE 1100 
#define PPM_MID 1500
#define PPM_MAX 2000
#define PPM_MIN_COMMAND 1300
#define PPM_MAX_COMMAND 1700
#define GET_FLAG(ch, mask) (PPM[ch] > PPM_MAX_COMMAND ? mask : 0)
#define GET_FLAG_INV(ch, mask) (PPM[ch] < PPM_MIN_COMMAND ? mask : 0)

struct {
    uint16_t volt1;
    uint16_t rssi;
    uint8_t  updated;
    uint32_t lastUpdate;
} telemetry_data;

static void ReadFlash(void);
static void WriteFlash(void);
static void ShowSticks(void);
static void CalibSticks(void);

void setup(){
  Serial.begin(115200);
  pinMode(LED_pin, OUTPUT); 
  
  pinMode(MO_pin  , OUTPUT);//SI 
  pinMode(MI_pin  , INPUT); //SO
  pinMode(SCLK_pin, OUTPUT);//SCLK SCK 
  pinMode(CS      , OUTPUT);//CS output
 
  CS_on;
  SCK_off;
  MO_off;

  delay(10);
  Serial.print("PartNum: ");
  Serial.println(cc2500_readReg(CC2500_30_PARTNUM + CC2500_READ_BURST));
  delay(10);
  Serial.print("Version: ");
  Serial.println(cc2500_readReg(CC2500_31_VERSION + CC2500_READ_BURST));
  
  frsky2way_init(1);
  state = FRSKY_BIND;//
  while(state<FRSKY_BIND_DONE){// 
    unsigned long pause;    
    pause=micros();
    cc2500_strobe(CC2500_SIDLE);
    cc2500_writeReg(CC2500_0A_CHANNR, 0x00);
    cc2500_writeReg(CC2500_23_FSCAL3, 0x89);
    frsky2way_build_bind_packet();
    cc2500_writeReg(CC2500_3E_PATABLE, 0x50);
    cc2500_strobe(CC2500_SFRX);//0x3A
    cc2500_writeFifo(packet, packet[0]+1);
    state++;
    while((micros()-pause)<9000);   
  }
  if(state==FRSKY_BIND_DONE){
    state=FRSKY_DATA2;
    frsky2way_init(0);
    counter=0;
  }
  LED_ON;
  
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

void loop(){
  unsigned long pause;
  pause=micros();
  if (state == FRSKY_DATA5){
    state = FRSKY_DATA1;  
    cc2500_strobe(CC2500_SRX);//0x34
    while((micros()-pause)<7450);
      return;
  }
  counter = (counter + 1) % 188;  
  if (state == FRSKY_DATA4) {
    //telemetry receive
    //CC2500_SetTxRxMode(RX_EN);
    cc2500_strobe(CC2500_SIDLE);
    cc2500_writeReg(CC2500_0A_CHANNR, get_chan_num(counter % 47));
    cc2500_writeReg(CC2500_23_FSCAL3, 0x89);
    state++;
    while((micros()-pause)<1450);
    return;
  }
  else 
  {
    if (state == FRSKY_DATA1) 
    {
      int len = cc2500_readReg(CC2500_3B_RXBYTES | CC2500_READ_BURST);
      if (len && len < 40) {
        cc2500_readFifo(packet, len);
        //parse telemetry packet here
      }
    }
    cc2500_strobe(CC2500_SIDLE);
    cc2500_writeReg(CC2500_0A_CHANNR, get_chan_num(counter % 47));
    cc2500_writeReg(CC2500_23_FSCAL3, 0x89);
    frsky2way_data_frame();
    cc2500_writeReg(CC2500_3E_PATABLE, 0xfe);
    cc2500_strobe(CC2500_SFRX);        
    cc2500_writeFifo(packet, packet[0]+1);
    state++;
  }       
#if defined(DEBUG)
  Serial.print(get_chan_num(counter % 47),HEX);
#endif    
  while((micros()-pause)<9000);
  update_ppm();
}

void update_ppm(){
//  Serial.print("S1:");    Serial.print(digitalRead(S1_PIN));
//  Serial.print("\tS2:");  Serial.print(digitalRead(S2_PIN));
//  Serial.print("\tB1:");  Serial.print(digitalRead(B1_PIN));
//  Serial.print("\tB2:");  Serial.print(digitalRead(B2_PIN));
//  Serial.print("\tB3:");  Serial.print(digitalRead(B3_PIN));
//  Serial.print("\tB4:");  Serial.println(digitalRead(B4_PIN));

  TRA.addValue(constrain(map(ReadT,0,TRangeMax,MaxMap,MinMap),SticksMin,SticksMax));
  RRA.addValue(constrain(map(ReadR,0,RRangeMax,MinMap,MaxMap),SticksMin,SticksMax));           
  PRA.addValue(constrain(map(ReadP,0,PRangeMax,MinMap,MaxMap),SticksMin,SticksMax));            
  YRA.addValue(constrain(map(ReadY,0,YRangeMax,MinMap,MaxMap),SticksMin,SticksMax)); 

  PPM[THROTTLE] = TRA.getFastAverage() + TOffSet;
  PPM[ROLL]     = RRA.getFastAverage() + ROffSet;
  PPM[PITCH]    = PRA.getFastAverage() + POffSet;
  PPM[YAW]      = YRA.getFastAverage() + YOffSet;

  PPM[ARM_AUX]  = digitalRead(S1_PIN) ? 1200 : 1800;
  PPM[ACRO_AUX] = digitalRead(S2_PIN) ? 1200 : 1800;
    

//  Serial.print("T:");      Serial.print(PPM[THROTTLE]);
//  Serial.print("\tR:");    Serial.print(PPM[ROLL]);
//  Serial.print("\tP:");    Serial.print(PPM[PITCH]);
//  Serial.print("\tY:");    Serial.print(PPM[YAW]);
//  Serial.print("\tAUX:");  Serial.println(PPM[FLIP_AUX]);

  if(bShowSticks && !bIsCalib)
  {
    ShowSticks();
  }
  else if(bIsCalib)
  {
    CalibSticks();
  }
  else
  {
    if(PPM[ROLL]  > 1485 && PPM[ROLL]  < 1515) PPM[ROLL]   = 1500;
    if(PPM[PITCH] > 1485 && PPM[PITCH] < 1515) PPM[PITCH]  = 1500;
    if(PPM[YAW]   > 1485 && PPM[YAW]   < 1515) PPM[YAW]    = 1500;
  }
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

static int convert_channel(uint8_t ch){
	float value;
	if(PPM[ch]>PPM_MAX)
	PPM[ch]=PPM_MAX;
	if (PPM[ch]<PPM_MIN)
	PPM[ch]=PPM_MIN;
	value = (float)PPM[ch] + PPM[ch]/2;
	return value;
}

static int get_chan_num(int idx)
{
  int ret = (idx * 0x1e) % 0xeb;
  if(idx == 3 || idx == 23 || idx == 47)
  {
    ret++;
  }
  if(idx > 47)
  {
    return 0;
  }
  return ret;
}

#include "iface_cc2500.h"

//adjust here yor frequency offset in case  you cannot binding with your CC2500 ,module   
static uint8_t fine = 0xd7;//* 215 *//give values from 0 to 255 for freq offset
                           //values from 0-127 offset increase frequency ,
                           //values from 255 to 127 decrease base frequency
                            //this is useful for tunning TX base frequency to frsky RX freq.
static uint32_t fixed_id = 0x3e19;
  //fixed_id = 0x3e19;
  //fixed_id = 0x2DD7;
  //fixed_id = 0x1EE9;
void frsky2way_init(uint8_t bind){
  cc2500_resetChip();
  cc2500_writeReg(CC2500_02_IOCFG0, 0x06);    
  cc2500_writeReg(CC2500_00_IOCFG2, 0x06);
  cc2500_writeReg(CC2500_17_MCSM1, 0x0c);
  cc2500_writeReg(CC2500_18_MCSM0, 0x18);
  cc2500_writeReg(CC2500_06_PKTLEN, 0x19);//25
  cc2500_writeReg(CC2500_07_PKTCTRL1, 0x04);
  cc2500_writeReg(CC2500_08_PKTCTRL0, 0x05);
  cc2500_writeReg(CC2500_3E_PATABLE, 0xff);
  cc2500_writeReg(CC2500_0B_FSCTRL1, 0x08);
  cc2500_writeReg(CC2500_0C_FSCTRL0, 0x00);
  cc2500_writeReg(CC2500_0D_FREQ2, 0x5c); 
  cc2500_writeReg(CC2500_0E_FREQ1, 0x76);
  cc2500_writeReg(CC2500_0F_FREQ0, 0x27);
  cc2500_writeReg(CC2500_10_MDMCFG4, 0xAA);   
  cc2500_writeReg(CC2500_11_MDMCFG3, 0x39);
  cc2500_writeReg(CC2500_12_MDMCFG2, 0x11);
  cc2500_writeReg(CC2500_13_MDMCFG1, 0x23);
  cc2500_writeReg(CC2500_14_MDMCFG0, 0x7a);
  cc2500_writeReg(CC2500_15_DEVIATN, 0x42);
  cc2500_writeReg(CC2500_19_FOCCFG, 0x16);
  cc2500_writeReg(CC2500_1A_BSCFG, 0x6c); 
  cc2500_writeReg(CC2500_1B_AGCCTRL2,0x03); 
  cc2500_writeReg(CC2500_1C_AGCCTRL1,0x40);
  cc2500_writeReg(CC2500_1D_AGCCTRL0,0x91);
  cc2500_writeReg(CC2500_21_FREND1, 0x56);
  cc2500_writeReg(CC2500_22_FREND0, 0x10);
  cc2500_writeReg(CC2500_23_FSCAL3, 0xa9);
  cc2500_writeReg(CC2500_24_FSCAL2, 0x0A);
  cc2500_writeReg(CC2500_25_FSCAL1, 0x00);
  cc2500_writeReg(CC2500_26_FSCAL0, 0x11);
  cc2500_writeReg(CC2500_29_FSTEST, 0x59);
  cc2500_writeReg(CC2500_2C_TEST2, 0x88);
  cc2500_writeReg(CC2500_2D_TEST1, 0x31);
  cc2500_writeReg(CC2500_2E_TEST0, 0x0B);
  cc2500_writeReg(CC2500_03_FIFOTHR, 0x07);
  cc2500_writeReg(CC2500_09_ADDR, 0x00);
  cc2500_strobe(CC2500_SIDLE);    
  cc2500_writeReg(CC2500_02_IOCFG0, 0x06);
  cc2500_writeReg(CC2500_09_ADDR, bind ? 0x03 : (fixed_id & 0xff));
  cc2500_writeReg(CC2500_07_PKTCTRL1, 0x04);      
  cc2500_writeReg(CC2500_0C_FSCTRL0, fine);
}
  
 
void frsky2way_build_bind_packet()
{
    //11 03 01 d7 2d 00 00 1e 3c 5b 78 00 00 00 00 00 00 01
    //11 03 01 19 3e 00 02 8e 2f bb 5c 00 00 00 00 00 00 01
  packet[0] = 0x11;                
  packet[1] = 0x03;                
  packet[2] = 0x01;                
  packet[3] = fixed_id & 0xff;
  packet[4] = fixed_id >> 8;
  int idx = ((state -FRSKY_BIND) % 10) * 5;
  packet[5] = idx;
  packet[6] = get_chan_num(idx++);
  packet[7] = get_chan_num(idx++);
  packet[8] = get_chan_num(idx++);
  packet[9] = get_chan_num(idx++);
  packet[10] = get_chan_num(idx++);
  packet[11] = 0x00;
  packet[12] = 0x00;
  packet[13] = 0x00;
  packet[14] = 0x00;
  packet[15] = 0x00;
  packet[16] = 0x00;
  packet[17] = 0x01;
#if defined(DEBUG4)
  for(int i=0;i<11;i++){
    Serial.print(" ");
    Serial.print(packet[i],HEX);
    Serial.print(" ");
  }
  Serial.println(" ");
#endif
}


void frsky2way_data_frame()
{
  packet[0] = 0x11;             //Length
  packet[1] = fixed_id & 0xff;
  packet[2] = fixed_id >> 8;
  packet[3] = counter;//
  packet[4] = 0x00;
  packet[5] = 0x01;
  //
  packet[10] = 0;
  packet[11] = 0;
  packet[16] = 0;
  packet[17] = 0;
  for(int i = 0; i < 8; i++) {
    int value;
    if(i >= 8) {
      value = 0x8ca;
    } 
    else
    {
      value = convert_channel(i);
    }
    if(i < 4) 
    {
      packet[6+i] = value & 0xff;
      packet[10+(i>>1)] |= ((value >> 8) & 0x0f) << (4 *(i & 0x01));
    } 
    else
    {
      packet[8+i] = value & 0xff;
      packet[16+((i-4)>>1)] |= ((value >> 8) & 0x0f) << (4 * ((i-4) & 0x01));
    }
  }
#if defined(DEBUG3)   
  for(int i = 0; i < 18; i++){
    Serial.print(" ");
    Serial.print(packet[i],HEX);
    Serial.print(" ");
  }
  Serial.println(" ");
#endif
}
