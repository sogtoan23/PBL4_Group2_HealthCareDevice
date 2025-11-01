#include "lmic_project_config.h"
#include <Arduino.h>
#include <SPI.h>
#include <lmic.h>
#include <hal/hal.h>
#include <DHT.h>
#include <esp_system.h>   // esp_random() cho jitter trên ESP32/ESP32-C3

/* ====== TTN KEYS ====== */
static const u1_t PROGMEM APPEUI[8] = { 0xCD, 0xAB, 0x00, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
static const u1_t PROGMEM DEVEUI[8] = { 0xE6, 0x4C, 0x00, 0xD8, 0x7E, 0xD5, 0xB3, 0x70 };
static const u1_t PROGMEM APPKEY[16]= { 0x14, 0x67, 0xDB, 0x5E, 0xC9, 0x4C, 0x46, 0xA6, 0x21, 0x5F, 0x04, 0xB4, 0x53, 0x44, 0x06, 0x92 };

void os_getArtEui(u1_t* b){ memcpy_P(b, APPEUI, 8); }
void os_getDevEui(u1_t* b){ memcpy_P(b, DEVEUI, 8); }
void os_getDevKey(u1_t* b){ memcpy_P(b, APPKEY,16); }

/* ====== ESP32-C3 SuperMini ↔ SX1276 (Ra-01H) ====== */
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

/* ====== DHT11 ====== */
#define DHTPIN   8
#define DHTTYPE  DHT11
DHT dht(DHTPIN, DHTTYPE);

/* ====== App Timing ====== */
static osjob_t sendjob;
static osjob_t wdjob;
const unsigned TX_INTERVAL_S           = 60;  // ≥15s cho ThingSpeak
const unsigned JITTER_MAX_S            = 5;   // ngẫu nhiên 1..5s
const unsigned PENDING_RETRY_S         = 5;
const unsigned JOIN_TIMEOUT_S          = 90;
const unsigned WATCHDOG_MAX_SILENCE_S  = 5 * TX_INTERVAL_S;

/* ====== State Tracking ====== */
volatile bool    g_joined = false;
static ostime_t  g_last_activity = 0;
static ostime_t  g_join_started  = 0;

static inline uint32_t sec_to_osticks(uint32_t s){ return sec2osticks(s); }

// thêm jitter 1..JITTER_MAX_S (nếu after_sec >= TX_INTERVAL_S)
static void schedule_send(uint32_t after_sec, bool add_jitter){
  uint32_t j = 0;
  if (add_jitter && after_sec >= TX_INTERVAL_S) {
    j = 1 + (esp_random() % JITTER_MAX_S); // 1..JITTER_MAX_S
  }
  os_setTimedCallback(&sendjob, os_getTime() + sec_to_osticks(after_sec + j), do_send);
}

static void buildPayload(int16_t t_x10, int16_t h_x10, uint8_t *out) {
  out[0] = (uint8_t)((t_x10 >> 8) & 0xFF);
  out[1] = (uint8_t)(t_x10 & 0xFF);
  out[2] = (uint8_t)((h_x10 >> 8) & 0xFF);
  out[3] = (uint8_t)(h_x10 & 0xFF);
}

static void watchdog_fn(osjob_t*){
  ostime_t now = os_getTime();
  if (g_last_activity == 0) g_last_activity = now;

  // JOIN timeout
  if (!g_joined && g_join_started != 0 &&
      (now - g_join_started) > sec_to_osticks(JOIN_TIMEOUT_S)) {
    Serial.println(F("[WD] JOIN timeout → reset & rejoin"));
    LMIC_reset();
    g_join_started = os_getTime();
    LMIC_startJoining();
  }

  // Im lặng quá lâu
  if ((now - g_last_activity) > sec_to_osticks(WATCHDOG_MAX_SILENCE_S)) {
    Serial.println(F("[WD] No TX activity → reset & rejoin"));
    LMIC_reset();
    g_joined = false;
    g_join_started = os_getTime();
    LMIC_startJoining();
    schedule_send(PENDING_RETRY_S, false);
  }

  os_setTimedCallback(&wdjob, now + sec_to_osticks(10), watchdog_fn);
}

void do_send(osjob_t*){
  if (!g_joined) { Serial.println(F("Not joined yet → retry")); schedule_send(PENDING_RETRY_S, false); return; }
  if (LMIC.opmode & OP_TXRXPEND){ Serial.println(F("OP_TXRXPEND → retry in 5s")); schedule_send(PENDING_RETRY_S, false); return; }

  float h = dht.readHumidity();
  float t = dht.readTemperature(); // °C
  if (isnan(h) || isnan(t)) {
    Serial.println(F("DHT read failed → skip & next interval"));
    schedule_send(TX_INTERVAL_S, true);
    return;
  }

  int16_t t_x10 = (int16_t)lroundf(t * 10.0f);
  int16_t h_x10 = (int16_t)lroundf(h * 10.0f);
  uint8_t payload[4];
  buildPayload(t_x10, h_x10, payload);

  LMIC_setTxData2(1, payload, sizeof(payload), 0); // FPort=1, unconfirmed
  Serial.printf("TX queued: T=%.1f°C (x10=%d), H=%.1f%% (x10=%d)\n", t, t_x10, h, h_x10);

  // hẹn lần sau với jitter để tránh đụng biên 15s ThingSpeak
  schedule_send(TX_INTERVAL_S, true);
}

void onEvent(ev_t ev){
  switch(ev){
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      g_joined = false;
      g_join_started = os_getTime();
      g_last_activity = os_getTime();
      break;

    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      g_joined = true;
      g_last_activity = os_getTime();
      LMIC_setAdrMode(0);
      LMIC_setDrTxpow(2 /*DR2*/, 14);
      schedule_send(1, false); // gửi sớm gói đầu
      break;

    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED → retry"));
      g_joined = false;
      g_join_started = os_getTime();
      schedule_send(PENDING_RETRY_S, false);
      break;

    case EV_TXSTART:
      g_last_activity = os_getTime();
      Serial.printf("EV_TXSTART  freq=%.1f MHz  DR=%d\n", LMIC.freq/1e6, LMIC.datarate);
      break;

    case EV_TXCANCELED:
      g_last_activity = os_getTime();
      Serial.println(F("EV_TXCANCELED → retry soon"));
      schedule_send(PENDING_RETRY_S, false);
      break;

    case EV_TXCOMPLETE:
      g_last_activity = os_getTime();
      Serial.println(F("EV_TXCOMPLETE"));
      if (LMIC.txrxFlags & TXRX_ACK) Serial.println(F("ACK received"));
      if (LMIC.dataLen){
        Serial.print(F("RX: "));
        Serial.write(LMIC.frame + LMIC.dataBeg, LMIC.dataLen);
        Serial.println();
      }
      break;

    default: break;
  }
}

void setup(){
  Serial.begin(115200);
  Serial.println(F("Starting..."));
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_NSS);
  dht.begin();

  os_init();
  LMIC_reset();
  LMIC_setClockError(MAX_CLOCK_ERROR * 15 / 100); // nới 15% cho C3
  LMIC_setAdrMode(0);
  LMIC_setDrTxpow(0 /*DR0*/, 14);                  // OTAA join chắc chắn

  g_joined = false;
  g_join_started = os_getTime();
  g_last_activity = os_getTime();

  os_setTimedCallback(&wdjob, os_getTime() + sec_to_osticks(10), watchdog_fn);
  LMIC_startJoining();
}

void loop(){ os_runloop_once(); }
