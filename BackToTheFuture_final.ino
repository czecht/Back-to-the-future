#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <RTClib.h>

// ===================== I2C ADDRESSES =====================
// 16x2 displays
#define LCD_ADDR_DEST   0x27
#define LCD_ADDR_PRES   0x26
#define LCD_ADDR_LAST   0x25
#define LCD_ADDR_SPEED  0x24
#define LCD_ADDR_PLUT   0x23
#define LCD_ADDR_SYS    0x21
// 20x4 FLUX
#define LCD_ADDR_FLUX   0x3F

// ===================== LCD OBJECTS =====================
LiquidCrystal_I2C lcdDest (LCD_ADDR_DEST, 16, 2);
LiquidCrystal_I2C lcdPres (LCD_ADDR_PRES, 16, 2);
LiquidCrystal_I2C lcdLast (LCD_ADDR_LAST, 16, 2);
LiquidCrystal_I2C lcdSpeed(LCD_ADDR_SPEED, 16, 2);
LiquidCrystal_I2C lcdPlut (LCD_ADDR_PLUT, 16, 2);
LiquidCrystal_I2C lcdSys  (LCD_ADDR_SYS,  16, 2);
LiquidCrystal_I2C lcdFlux (LCD_ADDR_FLUX, 20, 4);  // 2004A

// ===================== RTC =====================
RTC_DS3231 rtc;

// ===================== KEYPAD SETUP =====================
const byte ROWS = 4;
const byte COLS = 4;

char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};

// Working mapping from before (rows/cols swapped)
byte rowPins[ROWS] = {9, A0, A1, A2}; // R1-R4
byte colPins[COLS] = {2, 4, 7, 8};    // C1-C4

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// ===================== UI MODES =====================
enum UIMode { MODE_NORMAL, MODE_SET_RTC };
UIMode uiMode = MODE_NORMAL;

// Buffer for clock-set input MMDDYYYYHHMM
char rtcInput[13];
byte rtcInputIndex = 0;

// ===================== TIME CIRCUIT DEMO DATA =====================
// DESTINATION and LAST are still demo strings for now
const char* destTimes[] = {
  "NOV05 1955 06:00",
  "OCT21 2015 07:28",
  "JAN01 1885 12:00",
  "DEC31 1999 23:59"
};
const uint8_t NUM_DEST_TIMES = sizeof(destTimes) / sizeof(destTimes[0]);

char lastDepartTime[17] = "--- -- ---- --:--";
uint8_t destIndex = 0;

// ===================== TIMERS =====================
unsigned long lastDestUpdate   = 0;
unsigned long lastPresUpdate   = 0;
unsigned long lastSpeedUpdate  = 0;
unsigned long lastPlutUpdate   = 0;
unsigned long lastFluxUpdate   = 0;
unsigned long lastSysUpdate    = 0;

// ===================== SPEED / PLUT / FLUX STATE =====================
int   speedValue     = 0;
int   speedDirection = 1;
uint8_t plutLevel    = 0;
uint8_t fluxFrame    = 0;

// ===================== SYSTEM STATUS =====================
const char* sysMessages[] = {
  "SYS OK  ALL GREEN",
  "TEMP HIGH! WARN  ",
  "TIME STREAM STABLE",
  "PLUT LVL NOMINAL ",
  "ANOMALY DETECTED!"
};
const uint8_t NUM_SYS_MSG = sizeof(sysMessages) / sizeof(sysMessages[0]);
uint8_t sysIndex = 0;

// ===================================================================
// =============== BUTTON SEQUENCE (ARM / READY / GO / ABORT) =========
const uint8_t PIN_BTN_YELLOW = 10;
const uint8_t PIN_BTN_GREEN  = 11;
const uint8_t PIN_BTN_RED    = 12;

const uint8_t PIN_LED_YELLOW = 3;
const uint8_t PIN_LED_GREEN  = 5;
const uint8_t PIN_LED_RED    = 6;

enum GoState { ST_IDLE, ST_ARMED, ST_READY, ST_GO, ST_ABORT };
GoState goState = ST_IDLE;

unsigned long lastReadMs = 0;
const unsigned long DEBOUNCE_MS = 30;
bool yStable=1,gStable=1,rStable=1;

unsigned long greenPressStart = 0;
unsigned long fluxIntervalMs = 200;
unsigned long goStartMs = 0;

// ===================== SMALL HELPERS =====================
inline void ledWrite(uint8_t pin, uint8_t val) { analogWrite(pin, val); }
inline void ledsOff(){ ledWrite(PIN_LED_YELLOW,0); ledWrite(PIN_LED_GREEN,0); ledWrite(PIN_LED_RED,0); }

uint8_t triWave(unsigned long periodMs, uint8_t minV=0, uint8_t maxV=255){
  unsigned long t=millis()%periodMs,half=periodMs/2;
  uint8_t span=maxV-minV;
  if(t<half) return minV+(uint32_t)span*t/half;
  return minV+(uint32_t)span*(periodMs-t)/half;
}
uint8_t sqPulse(unsigned long periodMs,uint8_t dutyPct=50,uint8_t minV=0,uint8_t maxV=255){
  unsigned long t=millis()%periodMs;
  unsigned long on=(uint32_t)periodMs*dutyPct/100;
  return (t<on)?maxV:minV;
}

void sysMsg(const char* msg16){ lcdSys.setCursor(0,1); lcdSys.print(msg16); }
void showStateBanner(){
  lcdSys.setCursor(0,0);
  switch(goState){
    case ST_IDLE:  lcdSys.print(" TIME MACHINE  "); break;
    case ST_ARMED: lcdSys.print("   ARMED MODE  "); break;
    case ST_READY: lcdSys.print("   READY MODE  "); break;
    case ST_GO:    lcdSys.print("   GO!  GO!    "); break;
    case ST_ABORT: lcdSys.print(" *** ABORT *** "); break;
  }
}

// ===================== RTC HELPERS =====================
const char* monthName(uint8_t m){
  static const char* names[13] = {
    "ERR","JAN","FEB","MAR","APR","MAY","JUN","JUL","AUG","SEP","OCT","NOV","DEC"
  };
  if(m>12) return names[0];
  return names[m];
}

// Format: "MMMDD YYYY HH:MM" exactly 16 chars
void printDateLine(LiquidCrystal_I2C &lcd, const DateTime &dt){
  char buf[17];
  snprintf(buf, sizeof(buf), "%3s%02d %4d %02d:%02d",
           monthName(dt.month()),
           dt.day(),
           dt.year(),
           dt.hour(),
           dt.minute());
  lcd.print(buf);
}

// Parse MMDDYYYYHHMM → DateTime
bool parseRtcInputToDate(DateTime &dt){
  if(rtcInputIndex != 12) return false;
  int MM   = (rtcInput[0]-'0')*10 + (rtcInput[1]-'0');
  int DD   = (rtcInput[2]-'0')*10 + (rtcInput[3]-'0');
  int YYYY = (rtcInput[4]-'0')*1000 +
             (rtcInput[5]-'0')*100 +
             (rtcInput[6]-'0')*10 +
             (rtcInput[7]-'0');
  int HH   = (rtcInput[8]-'0')*10 + (rtcInput[9]-'0');
  int MIN  = (rtcInput[10]-'0')*10 + (rtcInput[11]-'0');

  // Basic sanity checks
  if(MM < 1 || MM > 12) return false;
  if(DD < 1 || DD > 31) return false;
  if(HH < 0 || HH > 23) return false;
  if(MIN < 0 || MIN > 59) return false;
  if(YYYY < 2000 || YYYY > 2099) return false; // DS3231 is 2000-based

  dt = DateTime(YYYY, MM, DD, HH, MIN, 0);
  return true;
}

// ===================== BUTTONS INIT & UPDATE =====================
void buttonsInit(){
  pinMode(PIN_BTN_YELLOW,INPUT_PULLUP);
  pinMode(PIN_BTN_GREEN,INPUT_PULLUP);
  pinMode(PIN_BTN_RED,INPUT_PULLUP);
  pinMode(PIN_LED_YELLOW,OUTPUT);
  pinMode(PIN_LED_GREEN,OUTPUT);
  pinMode(PIN_LED_RED,OUTPUT);
  ledsOff(); goState=ST_IDLE; fluxIntervalMs=200;
  showStateBanner();
  sysMsg("SYS READY       ");
}

void readButtonsDebounced(){
  unsigned long now=millis();
  if(now-lastReadMs<DEBOUNCE_MS) return;
  lastReadMs=now;
  yStable=digitalRead(PIN_BTN_YELLOW);
  gStable=digitalRead(PIN_BTN_GREEN);
  rStable=digitalRead(PIN_BTN_RED);
}

void buttonsUpdate(){
  readButtonsDebounced();
  bool yPressed=(yStable==LOW);
  bool gPressed=(gStable==LOW);
  bool rPressed=(rStable==LOW);

  // ABORT
  if(rPressed && goState!=ST_ABORT){
    goState=ST_ABORT; fluxIntervalMs=80;
    showStateBanner(); sysMsg("ABORTED         ");
  }

  switch(goState){
    case ST_IDLE:
      ledWrite(PIN_LED_YELLOW,triWave(2000,8,180));
      ledWrite(PIN_LED_GREEN,0);
      ledWrite(PIN_LED_RED,0);
      fluxIntervalMs=200;
      if(yPressed){ goState=ST_ARMED; showStateBanner(); sysMsg("ARMED           "); }
      break;

    case ST_ARMED:
      ledWrite(PIN_LED_YELLOW,200);
      ledWrite(PIN_LED_GREEN,triWave(1200,10,220));
      ledWrite(PIN_LED_RED,0);
      fluxIntervalMs=140;
      if(gPressed){ goState=ST_READY; greenPressStart=0; showStateBanner(); sysMsg("READY - HOLD G  "); }
      break;

    case ST_READY:
      ledWrite(PIN_LED_YELLOW,255);
      ledWrite(PIN_LED_GREEN,255);
      ledWrite(PIN_LED_RED,0);
      fluxIntervalMs=100;
      if(gPressed){
        if(greenPressStart==0) greenPressStart=millis();
        if(millis()-greenPressStart>=1000UL){
          goState=ST_GO; goStartMs=millis();
          showStateBanner(); sysMsg("GO! ACCELERATE! ");
          lcdFlux.setCursor(0,2); lcdFlux.print("   *** ACTIVE ***   ");
        }
      } else {
        greenPressStart=0;
      }
      break;

    case ST_GO:{
      uint8_t pulse=sqPulse(120,50,0,255);
      ledWrite(PIN_LED_YELLOW,pulse);
      ledWrite(PIN_LED_GREEN,pulse);
      ledWrite(PIN_LED_RED,0);
      fluxIntervalMs=40;

      if(millis()-goStartMs>=1500UL){
        lcdFlux.setCursor(0,2); lcdFlux.print("     STANDBY       ");
        goState=ST_IDLE; showStateBanner(); sysMsg("CYCLE COMPLETE  "); ledsOff();
      }
    } break;

    case ST_ABORT:
      ledWrite(PIN_LED_YELLOW,0);
      ledWrite(PIN_LED_GREEN,0);
      ledWrite(PIN_LED_RED,sqPulse(200,50,0,255));
      if(!rPressed){ goState=ST_IDLE; showStateBanner(); sysMsg("SYS READY       "); ledsOff();}
      break;
  }
}

// ===================== LCD INIT =====================
void initAllLCDs(){
  lcdDest.init(); lcdDest.backlight();
  lcdPres.init(); lcdPres.backlight();
  lcdLast.init(); lcdLast.backlight();
  lcdSpeed.init(); lcdSpeed.backlight();
  lcdPlut.init(); lcdPlut.backlight();
  lcdSys.init(); lcdSys.backlight();
  lcdFlux.init(); lcdFlux.backlight();

  byte fullBlock[8]={B11111,B11111,B11111,B11111,B11111,B11111,B11111,B11111};
  lcdPlut.createChar(0, fullBlock);
}

void initScreens(){
  lcdDest.clear(); lcdDest.setCursor(0,0); lcdDest.print("DESTINATION TIME");
  lcdDest.setCursor(0,1); lcdDest.print(destTimes[destIndex]);

  lcdPres.clear(); lcdPres.setCursor(0,0); lcdPres.print("PRESENT TIME   ");
  lcdPres.setCursor(0,1); lcdPres.print("RTC NOT READY  "); // overwritten soon

  lcdLast.clear(); lcdLast.setCursor(0,0); lcdLast.print("LAST TIME DEPART");
  lcdLast.setCursor(0,1); lcdLast.print(lastDepartTime);

  lcdSpeed.clear(); lcdSpeed.setCursor(0,0); lcdSpeed.print("SPD:   000 MPH");
  lcdSpeed.setCursor(0,1); lcdSpeed.print("--------------");

  lcdPlut.clear(); lcdPlut.setCursor(0,0); lcdPlut.print("PLUTONIUM LVL ");
  lcdPlut.setCursor(0,1); lcdPlut.print("[          ]");

  lcdFlux.clear();
  lcdFlux.setCursor(2,0); lcdFlux.print("FLUX CAPACITOR");
  lcdFlux.setCursor(0,1); lcdFlux.print("                    ");
  lcdFlux.setCursor(0,2); lcdFlux.print("        STANDBY     ");
  lcdFlux.setCursor(0,3); lcdFlux.print("                    ");

  lcdSys.clear();
  lcdSys.setCursor(0,0); lcdSys.print(" TIME MACHINE  ");
  lcdSys.setCursor(0,1); lcdSys.print("SYS INIT...    ");
}

// ===================== SPEEDOMETER =====================
void printThreeDigit(LiquidCrystal_I2C &lcd,int value){
  if(value<0)value=0; if(value>999)value=999;
  int h=value/100,t=(value/10)%10,o=value%10;
  if(h>0)lcd.print(h); else lcd.print(" ");
  if(h>0||t>0)lcd.print(t); else lcd.print(" ");
  lcd.print(o);
}

void updateSpeedometer(){
  speedValue+=speedDirection*4;
  if(speedValue>=88){speedValue=88; speedDirection=-1;}
  if(speedValue<=0){speedValue=0; speedDirection=1;}

  lcdSpeed.setCursor(5,0); printThreeDigit(lcdSpeed,speedValue);

  int barLength=map(speedValue,0,88,0,14);
  lcdSpeed.setCursor(0,1);
  for(int i=0;i<14;i++) lcdSpeed.print(i<barLength?">":"-");
}

// ===================== TIME CIRCUITS =====================
void updateTimeCircuits(){
  strncpy(lastDepartTime,destTimes[destIndex],16);
  lastDepartTime[16]='\0';
  destIndex=(destIndex+1)%NUM_DEST_TIMES;

  lcdDest.setCursor(0,1); lcdDest.print(destTimes[destIndex]);
  lcdLast.setCursor(0,1); lcdLast.print(lastDepartTime);
}

// PRESENT TIME from RTC
void updatePresentRTC(){
  DateTime now = rtc.now();
  lcdPres.setCursor(0,1);
  printDateLine(lcdPres, now);
}

// ===================== PLUTONIUM =====================
void drawPlutoniumBar(uint8_t level){
  lcdPlut.setCursor(1,1);
  for(uint8_t i=0;i<10;i++) lcdPlut.write(i<level?byte(0):' ');
}
void updatePlutonium(){
  plutLevel=(plutLevel+1)%11;
  drawPlutoniumBar(plutLevel);
}

// ===================== FLUX =====================
void updateFlux20x4(){
  fluxFrame=(fluxFrame+1)%4;
  for(int r=1;r<=3;r++){ lcdFlux.setCursor(0,r); lcdFlux.print("                    "); }

  switch(fluxFrame){
    case 0:
      lcdFlux.setCursor(4,1); lcdFlux.print("\\      |      /");
      lcdFlux.setCursor(5,2); lcdFlux.print("\\     *     / ");
      lcdFlux.setCursor(6,3); lcdFlux.print("\\    |    /  "); break;
    case 1:
      lcdFlux.setCursor(4,1); lcdFlux.print("\\      |      /");
      lcdFlux.setCursor(5,2); lcdFlux.print("\\    ***    / ");
      lcdFlux.setCursor(6,3); lcdFlux.print("\\    |    /  "); break;
    case 2:
      lcdFlux.setCursor(4,1); lcdFlux.print("\\      |      /");
      lcdFlux.setCursor(5,2); lcdFlux.print("\\   *****   / ");
      lcdFlux.setCursor(6,3); lcdFlux.print("\\    |    /  "); break;
    case 3:
      lcdFlux.setCursor(4,1); lcdFlux.print("\\      |      /");
      lcdFlux.setCursor(5,2); lcdFlux.print("\\    ***    / ");
      lcdFlux.setCursor(6,3); lcdFlux.print("\\    |    /  "); break;
  }
}

// ===================== SYSTEM =====================
void updateSystem(){
  sysIndex=(sysIndex+1)%NUM_SYS_MSG;
  lcdSys.setCursor(0,1); lcdSys.print(sysMessages[sysIndex]);
}

// ===================== KEYPAD HANDLING =====================

void enterRtcSetMode(){
  uiMode = MODE_SET_RTC;
  rtcInputIndex = 0;
  rtcInput[0] = '\0';

  lcdPres.clear();
  lcdPres.setCursor(0,0);
  lcdPres.print("SET PRESENT TIME");
  lcdPres.setCursor(0,1);
  lcdPres.print("MMDDYYYYHHMM   ");

  sysMsg("SET: #=OK *=CAN ");
}

void exitRtcSetMode(bool apply){
  if(apply){
    DateTime newDt;
    if(parseRtcInputToDate(newDt)){
      rtc.adjust(newDt);
      lcdPres.clear();
      lcdPres.setCursor(0,0);
      lcdPres.print("PRESENT TIME   ");
      updatePresentRTC();
      sysMsg("TIME UPDATED    ");
    } else {
      sysMsg("INVALID DATE    ");
      lcdPres.clear();
      lcdPres.setCursor(0,0);
      lcdPres.print("PRESENT TIME   ");
      updatePresentRTC();
    }
  } else {
    // Cancel: restore normal display
    lcdPres.clear();
    lcdPres.setCursor(0,0);
    lcdPres.print("PRESENT TIME   ");
    updatePresentRTC();
    sysMsg("CANCELLED       ");
  }

  uiMode = MODE_NORMAL;
}

// handle keypad globally
void handleKeypad(){
  char key = keypad.getKey();
  if(!key) return;

  if(uiMode == MODE_NORMAL){
    if(key == 'A'){
      enterRtcSetMode();
      return;
    }

    // normal-mode echo for other keys
    Serial.print("KEYPAD: ");
    Serial.println(key);
    lcdSys.setCursor(0,1);
    lcdSys.print("KEY: ");
    lcdSys.print(key);
    lcdSys.print("           ");

  } else if(uiMode == MODE_SET_RTC){
    if(key >= '0' && key <= '9'){
      if(rtcInputIndex < 12){
        rtcInput[rtcInputIndex++] = key;
        rtcInput[rtcInputIndex] = '\0';

        lcdPres.setCursor(0,1);
        lcdPres.print("                ");
        lcdPres.setCursor(0,1);
        lcdPres.print(rtcInput);
      }
    } else if(key == '*'){
      // cancel
      exitRtcSetMode(false);
    } else if(key == '#'){
      // confirm
      if(rtcInputIndex == 12){
        exitRtcSetMode(true);
      } else {
        sysMsg("NEED 12 DIGITS  ");
      }
    }
  }
}

// ===================== SETUP =====================
void setup(){
  Serial.begin(9600);
  Wire.begin();

  initAllLCDs();
  initScreens();

  // ----- RTC INIT -----
  if(!rtc.begin()){
    lcdPres.setCursor(0,1);
    lcdPres.print("RTC ERROR       ");
  } else {
    // Uncomment ONCE to set RTC from compile time, then comment and re-upload
    // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    updatePresentRTC();
  }

  buttonsInit();
}

// ===================== LOOP =====================
void loop(){
  unsigned long now=millis();

  // ---- KEYPAD ----
  handleKeypad();

  // ---- BUTTON SEQUENCE ----
  buttonsUpdate();

  // ---- TIME CIRCUITS ----
  if(now-lastDestUpdate>5000UL){ lastDestUpdate=now; updateTimeCircuits(); }

  // Only auto-update PRESENT TIME in normal mode
  if(now-lastPresUpdate>1000UL && uiMode==MODE_NORMAL){
    lastPresUpdate=now;
    updatePresentRTC();
  }

  // ---- SPEED ----
  if(now-lastSpeedUpdate>120UL){ lastSpeedUpdate=now; updateSpeedometer(); }

  // ---- PLUTONIUM ----
  if(now-lastPlutUpdate>400UL){ lastPlutUpdate=now; updatePlutonium(); }

  // ---- FLUX ----
  if(now-lastFluxUpdate>fluxIntervalMs){
    lastFluxUpdate=now;
    updateFlux20x4();
  }

  // ---- SYSTEM STATUS ----
  if(now-lastSysUpdate>4000UL){
    lastSysUpdate=now;
    if(uiMode==MODE_NORMAL){ // don't spam while setting clock
      updateSystem();
    }
  }
}
