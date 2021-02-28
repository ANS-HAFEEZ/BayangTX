#include <util/atomic.h>
#include <EEPROM.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "RunningAverage.h"
#include "PPMEncoder.h"
#include "pins.h"

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

void loop()
{
//  Serial.print("S1:");    Serial.print(digitalRead(S1_PIN));
//  Serial.print("\tS2:");  Serial.print(digitalRead(S2_PIN));
//  Serial.print("\tB1:");  Serial.print(digitalRead(B1_PIN));
//  Serial.print("\tB2:");  Serial.print(digitalRead(B2_PIN));
//  Serial.print("\tB3:");  Serial.print(digitalRead(B3_PIN));
//  Serial.print("\tB4:");  Serial.println(digitalRead(B4_PIN));

  TRA.addValue(constrain(map(analogRead(T_PIN),20,720,2000,1000),1050,1900));            
  RRA.addValue(constrain(map(analogRead(R_PIN),0,730,1000,2000),1050,1900));           
  PRA.addValue(constrain(map(analogRead(P_PIN),0,780,1000,2000),1050,1900));            
  YRA.addValue(constrain(map(analogRead(Y_PIN),0,780,1000,2000),1050,1900));  


  PPM[THROTTLE] = TRA.getFastAverage() + TOffSet;
  PPM[ROLL]     = RRA.getFastAverage() + ROffSet;
  PPM[PITCH]    = PRA.getFastAverage() + POffSet;
  PPM[RUDDER]   = YRA.getFastAverage() + YOffSet;

  PPM[ARM]      = digitalRead(S1_PIN) ? 1200 : 1800;
  PPM[ACRO]     = digitalRead(S2_PIN) ? 1200 : 1800;

//    Serial.print("R:");      Serial.print(PPM[ROLL]);
//    Serial.print("\tP:");    Serial.print(PPM[PITCH]);
//    Serial.print("\tT:");    Serial.print(PPM[THROTTLE]);
//    Serial.print("\tY:");    Serial.println(PPM[RUDDER]);

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
    if(PPM[THROTTLE]  < 1100){ 
      PPM[THROTTLE]   = 1100;
    }
    
    if(PPM[ROLL]  > 1480 && PPM[ROLL]  < 1520){ 
      PPM[ROLL]   = 1500;
    }
    if(PPM[PITCH] > 1480 && PPM[PITCH] < 1520){
      PPM[PITCH]  = 1500;
    }
    if(PPM[RUDDER]   > 1480 && PPM[RUDDER]   < 1520){
      PPM[RUDDER]    = 1500;
    }

//    Serial.print("R:");      Serial.print(PPM[ROLL]);
//    Serial.print("\tP:");    Serial.print(PPM[PITCH]);
//    Serial.print("\tT:");    Serial.print(PPM[THROTTLE]);
//    Serial.print("\tY:");    Serial.println(PPM[RUDDER]);

    ppmEncoder.setChannel(ROLL,     PPM[ROLL]);
    ppmEncoder.setChannel(PITCH,    PPM[PITCH]);
    ppmEncoder.setChannel(THROTTLE, PPM[THROTTLE]);
    ppmEncoder.setChannel(RUDDER,   PPM[RUDDER]);
    ppmEncoder.setChannel(ARM,      PPM[ARM]);
    ppmEncoder.setChannel(ACRO,     PPM[ACRO]);
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
