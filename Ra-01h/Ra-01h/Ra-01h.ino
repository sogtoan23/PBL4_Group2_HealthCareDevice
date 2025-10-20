#include "lmic_project_config.h"
#include <Arduino.h>
#include <SPI.h>
#include <lmic.h>
#include <hal/hal.h>

/* ====== TTN KEYS ======
 * AppEUI, DevEUI: LSB-first (đảo byte như TTN UI)
 * AppKey: MSB-first (giữ nguyên)
 */
static const u1_t PROGMEM APPEUI[8] = { 0xCD, 0xAB, 0x00, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
static const u1_t PROGMEM DEVEUI[8] = { 0xE6, 0x4C, 0x00, 0xD8, 0x7E, 0xD5, 0xB3, 0x70 };
static const u1_t PROGMEM APPKEY[16]= { 0x14, 0x67, 0xDB, 0x5E, 0xC9, 0x4C, 0x46, 0xA6, 0x21, 0x5F, 0x04, 0xB4, 0x53, 0x44, 0x06, 0x92 };

void os_getArtEui(u1_t* b){ memcpy_P(b, APPEUI, 8); }
void os_getDevEui(u1_t* b){ memcpy_P(b, DEVEUI, 8); }
void os_getDevKey(u1_t* b){ memcpy_P(b, APPKEY,16); }

/* ====== ESP32-C3 SuperMini ↔ SX1276 (Ra-01H) ======
 * SCK=IO4, MISO=IO5, MOSI=IO6, NSS=IO7, RST=IO3, DIO0=IO2, DIO1=IO1
 */
#define PIN_SCK  4
#define PIN_MISO 5
#define PIN_MOSI 6
#define PIN_NSS  7
#define PIN_RST  3
#define PIN_DIO0 2
#define PIN_DIO1 1

const lmic_pinmap lmic_pins = {
  .nss=PIN_NSS, .rxtx=LMIC_UNUSED_PIN, .rst=PIN_RST,
  .dio={PIN_DIO0, PIN_DIO1, LMIC_UNUSED_PIN},
};

/* ====== App ====== */
static osjob_t sendjob;
const unsigned TX_INTERVAL = 60; // giây

void do_send(osjob_t* j){
  if (!(LMIC.opmode & OP_TXRXPEND)){
    static uint8_t msg[] = "Hello Quang Hue";
    LMIC_setTxData2(1, msg, sizeof(msg)-1, 0);  // FPort=1, unconfirmed
  }
  os_setTimedCallback(j, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
}

void onEvent(ev_t ev){
  switch(ev){
    case EV_JOINING:  Serial.println(F("EV_JOINING")); break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      // Sau khi JOIN: chạy nhanh hơn ở DR2 (=SF10) — dùng chỉ số để khỏi lệ thuộc macro
      LMIC_setAdrMode(0);
      LMIC_setDrTxpow(2 /*DR2*/, 14);
      do_send(&sendjob);
      break;
    case EV_TXSTART:
      Serial.printf("EV_TXSTART  freq=%.1f MHz  DR=%d\n", LMIC.freq/1e6, LMIC.datarate);
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE"));
      if (LMIC.txrxFlags & TXRX_ACK) Serial.println(F("ACK received"));
      if (LMIC.dataLen){
        Serial.print(F("RX: "));
        Serial.write(LMIC.frame + LMIC.dataBeg, LMIC.dataLen);
        Serial.println();
      }
      break;
    case EV_JOIN_FAILED:  Serial.println(F("EV_JOIN_FAILED")); break;
    default: break;
  }
}

void setup(){
  Serial.begin(115200);
  Serial.println(F("Starting..."));

  // ESP32-C3: bắt buộc khởi tạo SPI với chân tự chọn
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_NSS);

  os_init();
  LMIC_reset();                                   // xoá phiên cũ
  LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100); // nới cửa sổ RX cho C3
  LMIC_setAdrMode(0);
  LMIC_setDrTxpow(0 /*DR0 = SF12*/, 14);          // JOIN bền chắc (không dùng DR_SF12)

  // Không gọi LMIC_setRx2Params — để LMIC dùng mặc định theo AS923 Group-1 & PHY 1.0.2

  LMIC_startJoining();
}

void loop(){ os_runloop_once(); }
