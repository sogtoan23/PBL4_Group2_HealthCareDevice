/************************************************************
 * ESP32-C3 HEALTHCARE WEARABLE – FULL FSM FINAL STABLE
 ************************************************************/

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <math.h>
#include <time.h>

#include <lmic.h>
#include <hal/hal.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "MAX30105.h"
#include <SparkFunLSM6DS3.h>

#include "icons.h"

/* ================= PIN ================= */
#define PIN_SCK   4
#define PIN_MISO  5
#define PIN_MOSI  6
#define PIN_NSS   7
#define PIN_RST   3
#define PIN_DIO0  2
#define PIN_DIO1  1

#define OLED_SDA  8
#define OLED_SCL  9

#define BTN_OK_PIN   21
#define BUZZER_PIN   10

/* ================= TTN OTAA ================= */
static const u1_t PROGMEM APPEUI[8]  ={0xCD,0xAB,0x00,0xD0,0x7E,0xD5,0xB3,0x70};
static const u1_t PROGMEM DEVEUI[8]  ={0xE6,0x4C,0x00,0xD8,0x7E,0xD5,0xB3,0x70};
static const u1_t PROGMEM APPKEY[16] ={0x14,0x67,0xDB,0x5E,0xC9,0x4C,0x46,0xA6,
                                      0x21,0x5F,0x04,0xB4,0x53,0x44,0x06,0x92};

void os_getArtEui(u1_t* buf){ memcpy_P(buf,APPEUI,8); }
void os_getDevEui(u1_t* buf){ memcpy_P(buf,DEVEUI,8); }
void os_getDevKey(u1_t* buf){ memcpy_P(buf,APPKEY,16); }

const lmic_pinmap lmic_pins={
  .nss=PIN_NSS,
  .rxtx=LMIC_UNUSED_PIN,
  .rst=PIN_RST,
  .dio={PIN_DIO0,PIN_DIO1,LMIC_UNUSED_PIN}
};

/* ================= OBJECT ================= */
Adafruit_SSD1306 display(128,64,&Wire,-1);
MAX30105 ppg;
LSM6DS3 imu(I2C_MODE,0x6B);

/* ================= FSM ================= */
enum SystemState {
  ST_WAIT_TIME,
  ST_WAIT_SETUP_BTN,
  ST_CALIB_FALL,
  ST_RUNNING,
  ST_FALL_ALARM
};
SystemState sysState = ST_WAIT_TIME;

/* ================= TIME ================= */
uint32_t unixBase=0, millisBase=0;
uint32_t nowUnix(){
  return unixBase ? unixBase + (millis()-millisBase)/1000 : 0;
}

/* ================= FALL ================= */
float Ab=0,Gb=0,At=0,Gt=0;
uint32_t calibStartMs=0;
uint16_t calibCount=0;

bool fall=false;
bool lockLoRa=false;
uint32_t fallConfirmStartMs=0;
#define FALL_CONFIRM_SEC 10
#define FALL_TX_PERIOD   10000
uint32_t lastFallTxMs=0;

/* ================= HR ================= */
bool ppgReady=false;
float bpm=0, spo2=0;
static uint32_t lastBeat=0;
static float dcIR=0,dcRed=0,acIR=0,acRed=0,th=0,ema=0;
static bool wasHigh=false;

/* ================= TIMERS ================= */
uint32_t lastHrTxMs=0;
uint32_t lastTimeReqMs=0;
uint32_t buzzerMs=0;
bool buzzerOn=false;

/* ================= INIT PPG ================= */
void initPPG(){
  if(ppgReady) return;
  if(!ppg.begin(Wire,I2C_SPEED_FAST)) return;
  ppg.setup(0x3F,4,2,400,411,16384);
  ppg.setPulseAmplitudeGreen(0);
  ppg.clearFIFO();
  ppgReady=true;
}

/* ================= UPDATE PPG ================= */
void updatePPG(){
  if(!ppgReady || sysState!=ST_RUNNING) return;

  ppg.check();
  while(ppg.available()){
    uint32_t ir=ppg.getIR(), red=ppg.getRed();
    dcIR=dcIR?0.99*dcIR+0.01*ir:ir;
    dcRed=dcRed?0.99*dcRed+0.01*red:red;

    float vIR=ir-dcIR;
    acIR=0.9*acIR+0.1*fabs(vIR);
    acRed=0.9*acRed+0.1*fabs(red-dcRed);

    th=0.8*th+0.2*(0.65*acIR);
    bool high=vIR>th;

    uint32_t now=millis();
    if(high&&!wasHigh&&lastBeat){
      uint32_t d=now-lastBeat;
      if(d>300&&d<2000){
        float inst=60000.0/d;
        ema=ema?0.8*ema+0.2*inst:inst;
        bpm=ema;
      }
    }
    if(high&&!wasHigh) lastBeat=now;
    wasHigh=high;

    if(dcIR>1000&&dcRed>1000&&acIR>1&&acRed>1){
      float R=(acRed/dcRed)/(acIR/dcIR);
      spo2=constrain(110-25*R,0,100);
    }
    ppg.nextSample();
  }
}

/* ================= PORT 2 ================= */
void sendPort2_RequestTime(){
  if(LMIC.opmode & OP_TXRXPEND) return;
  uint8_t p=1;
  LMIC_setTxData2(2,&p,1,0);
}

/* ================= FSM ================= */
void updateFSM(){

  if(sysState==ST_WAIT_TIME){
    if(unixBase!=0) sysState=ST_WAIT_SETUP_BTN;
    return;
  }

  if(sysState==ST_WAIT_SETUP_BTN){
    if(digitalRead(BTN_OK_PIN)==LOW){
      Ab=Gb=0;
      calibCount=0;
      calibStartMs=millis();
      sysState=ST_CALIB_FALL;
      delay(300);
    }
    return;
  }

  if(sysState==ST_CALIB_FALL){
    float ax=imu.readFloatAccelX(), ay=imu.readFloatAccelY(), az=imu.readFloatAccelZ();
    float gx=imu.readFloatGyroX(), gy=imu.readFloatGyroY(), gz=imu.readFloatGyroZ();
    Ab+=sqrt(ax*ax+ay*ay+az*az);
    Gb+=sqrt(gx*gx+gy*gy+gz*gz);
    calibCount++;
    if(millis()-calibStartMs>=5000){
      Ab/=calibCount; Gb/=calibCount;
      At=Ab+0.8; Gt=Gb+150;
      initPPG();
      sysState=ST_RUNNING;
    }
    return;
  }

  if(sysState==ST_RUNNING){
    float ax=imu.readFloatAccelX(), ay=imu.readFloatAccelY(), az=imu.readFloatAccelZ();
    float gx=imu.readFloatGyroX(), gy=imu.readFloatGyroY(), gz=imu.readFloatGyroZ();
    float A=sqrt(ax*ax+ay*ay+az*az);
    float G=sqrt(gx*gx+gy*gy+gz*gz);

    if(A>At && G>Gt){
      fall=true;
      lockLoRa=false;
      fallConfirmStartMs=millis();
      sysState=ST_FALL_ALARM;
    }
    return;
  }

  if(sysState==ST_FALL_ALARM){
    if(digitalRead(BTN_OK_PIN)==LOW){
      fall=false;
      lockLoRa=false;
      digitalWrite(BUZZER_PIN,LOW);
      sysState=ST_RUNNING;
      delay(300);
      return;
    }
    if(millis()-fallConfirmStartMs>=FALL_CONFIRM_SEC*1000){
      lockLoRa=true;
    }
  }
}

/* ================= BUZZER ================= */
void updateBuzzer(){
  if(sysState!=ST_FALL_ALARM){
    digitalWrite(BUZZER_PIN,LOW);
    return;
  }
  if(millis()-buzzerMs>1000){
    buzzerMs=millis();
    buzzerOn=!buzzerOn;
    digitalWrite(BUZZER_PIN,buzzerOn);
  }
}

/* ================= OLED ================= */
void updateOLED(){
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.drawBitmap(104,0,
    (LMIC.devaddr!=0)?epd_bitmap_wifi:epd_bitmap_no_conn,
    24,24,1);

  if(sysState==ST_WAIT_TIME){
    display.setTextSize(2);
    display.setCursor(32,24);
    display.print("WAIT...");
    display.display(); return;
  }

  if(sysState==ST_WAIT_SETUP_BTN){
    display.setTextSize(1);
    display.setCursor(14,22);
    display.print("Stand straight");
    display.setCursor(14,36);
    display.print("Press OK");
    display.display(); return;
  }

  if(sysState==ST_CALIB_FALL){
    int sec=5-(millis()-calibStartMs)/1000;
    display.setTextSize(1);
    display.setCursor(30,18);
    display.print("Calibrating");
    display.setTextSize(3);
    display.setCursor(54,36);
    display.print(max(0,sec));
    display.display(); return;
  }

  if(sysState==ST_FALL_ALARM){
    int remain=FALL_CONFIRM_SEC-(millis()-fallConfirmStartMs)/1000;
    if(remain<0) remain=0;
    display.drawBitmap(52,6,epd_bitmap_nofitication,24,24,1);
    display.setTextSize(2);
    display.setCursor(24,28);
    display.print("FALL");
    display.setCursor(56,48);
    display.print(remain);
    display.display(); return;
  }

  display.drawBitmap(0,0,epd_bitmap_heart_rate,24,24,1);
  display.drawBitmap(0,28,epd_bitmap_spo2,24,24,1);
  display.setTextSize(2);
  display.setCursor(30,12); display.print((int)bpm);
  display.setCursor(30,40); display.print((int)spo2);

  display.setTextSize(1);
  display.setCursor(0,56);
  time_t t=nowUnix()+7*3600;
  struct tm tm; gmtime_r(&t,&tm);
  char buf[24];
  snprintf(buf,sizeof(buf),
    "%02d/%02d/%04d %02d:%02d:%02d",
    tm.tm_mday,tm.tm_mon+1,tm.tm_year+1900,
    tm.tm_hour,tm.tm_min,tm.tm_sec);
  display.print(buf);
  display.display();
}

/* ================= LORA EVENT ================= */
void onEvent(ev_t ev){
  if(ev==EV_JOINED){
    LMIC_setLinkCheckMode(0);
  }
  if(ev==EV_TXCOMPLETE && LMIC.dataLen==4){
    uint8_t* b=LMIC.frame+LMIC.dataBeg;
    unixBase=((uint32_t)b[0]<<24)|((uint32_t)b[1]<<16)|((uint32_t)b[2]<<8)|b[3];
    millisBase=millis();
  }
}

/* ================= SETUP ================= */
void setup(){
  Serial.begin(115200);
  Wire.begin(OLED_SDA,OLED_SCL);
  display.begin(SSD1306_SWITCHCAPVCC,0x3C);

  pinMode(BTN_OK_PIN,INPUT_PULLUP);
  pinMode(BUZZER_PIN,OUTPUT);

  imu.begin();

  SPI.begin(PIN_SCK,PIN_MISO,PIN_MOSI,PIN_NSS);
  os_init(); LMIC_reset();
  LMIC_setDrTxpow(DR_SF10,14);
  LMIC_startJoining();
}

/* ================= LOOP ================= */
void loop(){
  os_runloop_once();
  updateFSM();
  updatePPG();
  updateBuzzer();

  if(sysState==ST_WAIT_TIME && unixBase==0 && !lockLoRa){
    if(millis()-lastTimeReqMs>15000){
      lastTimeReqMs=millis();
      sendPort2_RequestTime();
    }
  }

  if(sysState==ST_RUNNING && ppgReady && !lockLoRa){
    if(millis()-lastHrTxMs>30000){
      lastHrTxMs=millis();
      uint16_t b=(uint16_t)(bpm*10);
      uint16_t s=(uint16_t)(spo2*10);
      uint8_t p[4]={b>>8,b,s>>8,s};
      LMIC_setTxData2(1,p,4,0);
    }
  }

  if(sysState==ST_FALL_ALARM && lockLoRa){
    if(millis()-lastFallTxMs>FALL_TX_PERIOD){
      lastFallTxMs=millis();
      if(!(LMIC.opmode & OP_TXRXPEND)){
        uint16_t b=(uint16_t)bpm;
        uint8_t p[3]={1,(uint8_t)(b>>8),(uint8_t)b};
        LMIC_setTxData2(3,p,3,0);
      }
    }
  }

  static uint32_t t=0;
  if(millis()-t>300){
    t=millis();
    updateOLED();
  }
}
