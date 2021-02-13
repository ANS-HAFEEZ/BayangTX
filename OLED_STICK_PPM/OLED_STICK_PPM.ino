#include <util/atomic.h>
#include <EEPROM.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "RunningAverage.h"
#include "PPMEncoder.h"
#include "pins.h"
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//#define INPUT_FREQUENCY 50
//
//#define CHANNEL_MAX  2000
//#define CHANNEL_MIN 1000
//#define CHANNEL_MID 1500
//#define CHANNEL_NUMBER 7
//
//#define ANALOG_INPUTS 4
//#define DIGITAL_INPUTS 2
//
//#define FRAME_LENGTH 22500  //set the PPM frame length in microseconds (1ms = 1000µs)
//#define PULSE_LENGTH 300  //set the pulse length
//#define onState 1  //set polarity of the pulses: 1 is positive, 0 is negative
//#define sigPin 3  //set PPM signal output pin on the arduino
//
//int analogInputPins[ANALOG_INPUTS] = {R_PIN, P_PIN, T_PIN, Y_PIN};
//int digitalInputPins[DIGITAL_INPUTS] = {S1_PIN, S2_PIN};
//int switchPins[1] = {10};
//
//int channel_input[ANALOG_INPUTS]  = {};
//int channel_center[ANALOG_INPUTS] = {};
//int channel_min[ANALOG_INPUTS]    = {};
//int channel_max[ANALOG_INPUTS]    = {};
//
//int PPM[CHANNEL_NUMBER];
//
//int prevSwitch = LOW;
//byte currentState = LOW;
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


#define CHANNELS 8 // number of channels in ppm stream, 12 ideally
#define PPM_PIN 10
enum chan_order{
    ROLL,
    PITCH,
    THROTTLE,
    RUDDER,
    ARM,  
    ACRO, 
    AUX1, 
    AUX2
};

#define PPM_MIN 1000
#define PPM_SAFE_THROTTLE 1100 
#define PPM_MID 1500
#define PPM_MAX 2000
#define PPM_MIN_COMMAND 1300
#define PPM_MAX_COMMAND 1700

static uint16_t PPM[CHANNELS] = {PPM_MIN,PPM_MID,PPM_MID,PPM_MID,PPM_MID,PPM_MID,PPM_MID,PPM_MID};//,PPM_MID,PPM_MID,PPM_MID,PPM_MID,PPM_MID,PPM_MID,PPM_MID,PPM_MID};

static void ReadFlash(void);
static void WriteFlash(void);
static void ShowSticks(void);
static void CalibSticks(void);

void setup(){
  Serial.begin(115200);
  ppmEncoder.begin(PPM_PIN);
  if(!display.begin()) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }



//  //initiallize default ppm values
//  for (int i=0; i<CHANNEL_NUMBER; i++){
//      PPM[i]= CHANNEL_MID;
//  }
//
//  PPM[CHANNEL_NUMBER - 1] = 1000;
//
//  for (int i = 0; i < sizeof(digitalInputPins); i++) {
//    pinMode(digitalInputPins[i], INPUT_PULLUP);
//  }
//
//  for (int i = 0; i < ANALOG_INPUTS; i++) {
//    channel_center[i] = analogRead(analogInputPins[i]);
//  }
//
//  pinMode(switchPins[0], INPUT_PULLUP);
//
//  pinMode(sigPin, OUTPUT);
//  digitalWrite(sigPin, !onState);  //set the PPM signal pin to the default state (off)
//
//  cli();
//  TCCR1A = 0; // set entire TCCR1 register to 0
//  TCCR1B = 0;
//  
//  OCR1A = 100;  // compare match register, change this
//  TCCR1B |= (1 << WGM12);  // turn on CTC mode
//  TCCR1B |= (1 << CS11);  // 8 prescaler: 0,5 microseconds at 16mhz
//  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
//  sei();
  


  


  
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


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//void process_analog_channel(int channel) {
//  channel_input[channel] = analogRead(analogInputPins[channel]);
//
//  int diff = channel_center[channel] - channel_input[channel];
//
//  if (diff > channel_max[channel]) {
//    channel_max[channel] = diff;
//  }
//
//  if (diff < channel_min[channel]) {
//    channel_min[channel] = diff;
//  }
//
//  if (diff > 0) {
//    PPM[channel] = map(diff, 0, channel_max[channel], CHANNEL_MID, CHANNEL_MAX);
//  } else {
//    PPM[channel] = map(diff, channel_min[channel], 0, CHANNEL_MIN, CHANNEL_MID);
//  }
//  
//}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop()
{
//  Serial.print("S1:");    Serial.print(digitalRead(S1_PIN));
//  Serial.print("\tS2:");  Serial.print(digitalRead(S2_PIN));
//  Serial.print("\tB1:");  Serial.print(digitalRead(B1_PIN));
//  Serial.print("\tB2:");  Serial.print(digitalRead(B2_PIN));
//  Serial.print("\tB3:");  Serial.print(digitalRead(B3_PIN));
//  Serial.print("\tB4:");  Serial.println(digitalRead(B4_PIN));

  TRA.addValue(constrain(map(analogRead(T_PIN),0,600,2000,1000),1050,1900));            
  RRA.addValue(constrain(map(analogRead(R_PIN),0,730,1000,2000),1050,1900));           
  PRA.addValue(constrain(map(analogRead(P_PIN),0,800,1000,2000),1050,1900));            
  YRA.addValue(constrain(map(analogRead(Y_PIN),0,780,1000,2000),1050,1900));  


  PPM[THROTTLE] = TRA.getFastAverage() + TOffSet;
  PPM[ROLL]     = RRA.getFastAverage() + ROffSet;
  PPM[PITCH]    = PRA.getFastAverage() + POffSet;
  PPM[RUDDER]   = YRA.getFastAverage() + YOffSet;

  PPM[ARM]      = digitalRead(S1_PIN) ? 1200 : 1800;
  PPM[ACRO]     = digitalRead(S2_PIN) ? 1200 : 1800;

  PPM[AUX1] = 1200;
  PPM[AUX2] = 1800;

  ////////////////////////////////////////////////////////////////////////////////////////////////////
//    for (int i = 0; i < ANALOG_INPUTS; i++) {
//    process_analog_channel(i);
//  }
//
//  for (int i = 0; i < DIGITAL_INPUTS; i++) {
//    if (digitalRead(digitalInputPins[i]) == LOW) {
//      PPM[i + ANALOG_INPUTS] = 2000;
//    } else {
//      PPM[i + ANALOG_INPUTS] = 1000;
//    }
//  }
//
//  int swPin = digitalRead(switchPins[0]);
//
//  if (swPin == LOW && prevSwitch == HIGH) {
//    currentState = !currentState;
//  }
//
//  if (currentState == HIGH) {
//    PPM[CHANNEL_NUMBER - 1] = 2000;
//  } else {
//    PPM[CHANNEL_NUMBER - 1] = 1000;
//  }
//
//  prevSwitch = swPin;
//  
//  delay(1000 / INPUT_FREQUENCY);
  ////////////////////////////////////////////////////////////////////////////////////////////////////















  Serial.print("R:");      Serial.print(PPM[ROLL]);
  Serial.print("\tP:");    Serial.print(PPM[PITCH]);
  Serial.print("\tT:");    Serial.print(PPM[THROTTLE]);
  Serial.print("\tY:");    Serial.print(PPM[RUDDER]);
  Serial.print("\tAUX:");  Serial.println(PPM[ACRO]);

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
    if(PPM[ROLL]  > 1465 && PPM[ROLL]  < 1495) PPM[ROLL]   = 1481;
    if(PPM[PITCH] > 1465 && PPM[PITCH] < 1495) PPM[PITCH]  = 1481;
    if(PPM[RUDDER]   > 1465 && PPM[RUDDER]   < 1495) PPM[RUDDER]    = 1481;

    ppmEncoder.setChannel(ROLL,     PPM[ROLL]);
    ppmEncoder.setChannel(PITCH,    PPM[PITCH]);
    ppmEncoder.setChannel(THROTTLE, PPM[THROTTLE]);
    ppmEncoder.setChannel(RUDDER,   PPM[RUDDER]);
    ppmEncoder.setChannel(ARM,      PPM[ARM]);
    ppmEncoder.setChannel(ACRO,     PPM[ACRO]);
    ppmEncoder.setChannel(AUX1,     PPM[AUX1]);
    ppmEncoder.setChannel(AUX2,     PPM[AUX2]);
  }
}

//ISR(TIMER1_COMPA_vect){  //leave this alone
//  static boolean state = true;
//  
//  TCNT1 = 0;
//  
//  if (state) {  //start pulse
//    digitalWrite(sigPin, onState);
//    OCR1A = PULSE_LENGTH * 2;
//    state = false;
//  } else{  //end pulse and calculate when to start the next pulse
//    static byte cur_chan_numb;
//    static unsigned int calc_rest;
//  
//    digitalWrite(sigPin, !onState);
//    state = true;
//
//    if(cur_chan_numb >= CHANNEL_NUMBER){
//      cur_chan_numb = 0;
//      calc_rest = calc_rest + PULSE_LENGTH;// 
//      OCR1A = (FRAME_LENGTH - calc_rest) * 2;
//      calc_rest = 0;
//    }
//    else{
//      OCR1A = (PPM[cur_chan_numb] - PULSE_LENGTH) * 2;
//      calc_rest = calc_rest + PPM[cur_chan_numb];
//      cur_chan_numb++;
//    }     
//  }
//}


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
    display.setCursor(22,45); display.print(PPM[RUDDER]  );
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
    else if(stick==RUDDER){
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
      display.setCursor(22,0);  display.print(PPM[RUDDER]);
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
