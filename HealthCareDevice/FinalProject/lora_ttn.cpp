#include "lora_ttn.h"

#include "lmic_project_config.h"
#include <SPI.h>
#include <lmic.h>
#include <hal/hal.h>
#include <esp_system.h>

#include "ppg_sensor.h"
#include "downlink.h"

//===================== TTN KEYS =====================
static const u1_t PROGMEM APPEUI[8]  = { 0xCD, 0xAB, 0x00, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
static const u1_t PROGMEM DEVEUI[8]  = { 0xE6, 0x4C, 0x00, 0xD8, 0x7E, 0xD5, 0xB3, 0x70 };
static const u1_t PROGMEM APPKEY[16] = {
  0x14, 0x67, 0xDB, 0x5E, 0xC9, 0x4C, 0x46, 0xA6,
  0x21, 0x5F, 0x04, 0xB4, 0x53, 0x44, 0x06, 0x92
};

void os_getArtEui(u1_t* b){ memcpy_P(b, APPEUI, 8); }
void os_getDevEui(u1_t* b){ memcpy_P(b, DEVEUI, 8); }
void os_getDevKey(u1_t* b){ memcpy_P(b, APPKEY,16); }

//================= ESP32-C3 ↔ SX1276 =================
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

//================= Uplink timing (LMIC) ==============
static osjob_t sendjob;
const unsigned TX_INTERVAL_S   = 10;
const unsigned JITTER_MAX_S    = 3;
const unsigned PENDING_RETRY_S = 5;
volatile bool g_joined         = false;

static inline uint32_t sec_to_osticks(uint32_t s){ return sec2osticks(s); }
static void do_send(osjob_t*);

static void schedule_send(uint32_t after_sec, bool add_jitter){
  uint32_t j = (add_jitter && after_sec >= TX_INTERVAL_S)
               ? (1 + (esp_random() % JITTER_MAX_S)) : 0;

  os_setTimedCallback(
    &sendjob,
    os_getTime() + sec_to_osticks(after_sec + j),
    do_send
  );
}

//================= Payload pack ======================
static void buildPayload_u16x10(float bpm, float spo2, uint8_t *out) {
  auto enc = [](float v)->uint16_t {
    if (isnan(v)) v = 0;
    if (v < 0) v = 0; if (v > 6553.5f) v = 6553.5f;
    return (uint16_t)lroundf(v * 10.0f);
  };
  uint16_t b = enc(bpm);
  uint16_t s = enc(spo2);
  out[0] = (b >> 8) & 0xFF; out[1] = b & 0xFF;
  out[2] = (s >> 8) & 0xFF; out[3] = s & 0xFF;
}

//================= Uplink ============================
static void do_send(osjob_t*) {
  if (!g_joined) {
    schedule_send(PENDING_RETRY_S, false);
    return;
  }

  if (LMIC.opmode & OP_TXRXPEND) {
    schedule_send(PENDING_RETRY_S, false);
    return;
  }

  float bpm  = isnan(ppg_get_bpm())  ? 0.0f : ppg_get_bpm();
  float spo2 = isnan(ppg_get_spo2()) ? 0.0f : ppg_get_spo2();

  uint8_t payload[4];
  buildPayload_u16x10(bpm, spo2, payload);
  LMIC_setTxData2(1, payload, sizeof(payload), 0);

  Serial.printf("TX queued: BPM=%.1f SpO2=%.1f\n", bpm, spo2);

  schedule_send(TX_INTERVAL_S, true);
}

//================= LMIC events =======================
void onEvent(ev_t ev){
  switch(ev){

    case EV_JOINING:
      Serial.println("EV_JOINING");
      g_joined = false;
      break;

    case EV_JOINED:
      Serial.println("EV_JOINED");
      g_joined = true;

      // Sau khi join xong thì gửi uplink đầu tiên
      schedule_send(1, false);
      break;

    case EV_TXCOMPLETE:
      Serial.println("EV_TXCOMPLETE");

      // Nếu có downlink thì đưa cho handleDownlink xử lý (RTC, config,…)
      if (LMIC.dataLen) {
        uint8_t port = LMIC.frame[LMIC.dataBeg - 1];
        handleDownlink(port, LMIC.frame + LMIC.dataBeg, LMIC.dataLen);
      }
      break;

    default:
      break;
  }
}

//================= API cho main ======================
void lora_init() {
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_NSS);
  os_init();
  LMIC_reset();

  LMIC_setClockError(MAX_CLOCK_ERROR * 30 / 100);
  LMIC_setLinkCheckMode(0);

  LMIC_setAdrMode(0);
  LMIC_setDrTxpow(0, 14);
  LMIC_startJoining();
}

void lora_process() {
  os_runloop_once();
}
