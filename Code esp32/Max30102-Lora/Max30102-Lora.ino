/****************  ESP32-C3 + MAX30102 + OLED + TTN time  ****************
 * - Per-beat BPM + SpO2 (không smoothing), có quality gates & autogain
 * - Hiển thị OLED 0.96" I2C SSD1306 128x64
 * - Đồng bộ ngày giờ từ TTN bằng MAC Command DeviceTimeReq (MCCI LMIC)
 *************************************************************************/

#include "lmic_project_config.h"
#include <Arduino.h>
#include <SPI.h>
#include <lmic.h>
#include <hal/hal.h>
#include <esp_system.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>
#include <stdint.h>

//===================== TTN KEYS =====================
static const u1_t PROGMEM APPEUI[8]  = { 0xCD, 0xAB, 0x00, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
static const u1_t PROGMEM DEVEUI[8]  = { 0xE6, 0x4C, 0x00, 0xD8, 0x7E, 0xD5, 0xB3, 0x70 };
static const u1_t PROGMEM APPKEY[16] = { 0x14, 0x67, 0xDB, 0x5E, 0xC9, 0x4C, 0x46, 0xA6,
                                         0x21, 0x5F, 0x04, 0xB4, 0x53, 0x44, 0x06, 0x92 };
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

//================= MAX30102 (I2C) ====================
TwoWire I2C_0 = TwoWire(0);
#define I2C_SDA  8
#define I2C_SCL  9
MAX30105 ppg;

//================= OLED SSD1306 ======================
#define OLED_ADDR 0x3C
#define OLED_W    128
#define OLED_H    64
Adafruit_SSD1306 display(OLED_W, OLED_H, &I2C_0, -1);

//================= Cấu hình cảm biến =================
#define SAMPLE_RATE_HZ   100
#define PULSE_WIDTH_US   411
uint8_t LED_IR  = 0x30;
uint8_t LED_RED = 0x60;

#define ENABLE_AUTOGAIN  1
#define DC_TARGET_MIN    20000.0f
#define DC_TARGET_MAX    120000.0f
#define LED_MIN          0x10
#define LED_MAX          0x7F
#define LED_STEP_UP      0x08
#define LED_STEP_DOWN    0x08

#define FINGER_THRESH    5000L
#define MIN_BPM          40.0f
#define MAX_BPM          180.0f

// Quality gates
#define MIN_AC_IR        100.0f
#define MIN_AC_RED       80.0f
#define MIN_PI_IR        0.0010f
#define MIN_PI_RED       0.0008f
#define R_MIN            0.10f
#define R_MAX            2.50f
#define BEAT_MS_MIN      300
#define BEAT_MS_MAX      1500
#define DC_MIN           1.0f
#define DC_MAX           800000.0f
#define D_IR_MAX         120000.0f

// Hiệu chuẩn SpO2 = A - B*R
#define CAL_A            110.0f
#define CAL_B            12.0f

//================= Biến PPG per-beat =================
unsigned long lastBeatMs = 0;
float bpmInstant = NAN;
float spo2PerBeat = NAN;

bool  haveBeatWindow = false;
long  irMin = LONG_MAX, irMax = LONG_MIN;
long  redMin = LONG_MAX, redMax = LONG_MIN;
long  lastIR = 0;
float dIRmaxAbs = 0.0f;
uint16_t samplesInBeat = 0;

//================= Thời gian mạng (TTN) ==============
#define TZ_OFFSET_SECONDS   (7 * 3600)   // Việt Nam = UTC+7
static bool     haveNetTime = false;
static uint64_t epoch_base  = 0;       // epoch UTC lúc nhận
static uint32_t ms_anchor   = 0;       // millis() lúc nhận

//================= Uplink timing (LMIC) ==============
static osjob_t sendjob;
const unsigned TX_INTERVAL_S = 10;
const unsigned JITTER_MAX_S = 3;
const unsigned PENDING_RETRY_S = 5;
volatile bool g_joined = false;

// ==== DeviceTimeReq: đồng bộ lại định kỳ ====
static uint32_t uplinkCount = 0;
// 60 uplinks * 10s ≈ 10 phút
static const uint32_t RESYNC_EVERY_N_UPLINKS = 1;

static inline uint32_t sec_to_osticks(uint32_t s){ return sec2osticks(s); }
static void do_send(osjob_t*);

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

//================= Time formatting ===================
static String timeStringFromEpoch() {
  if (!haveNetTime) {
    unsigned long s = millis()/1000;
    uint16_t hh = (s/3600)%24, mm = (s/60)%60, ss = s%60;
    char buf[20];
    snprintf(buf,sizeof(buf),"--/-- %02u:%02u:%02u",hh,mm,ss);
    return String(buf);
  }
  uint32_t now_ms   = millis();
  uint64_t nowEpoch = epoch_base + (uint64_t)(now_ms - ms_anchor)/1000ULL + (int32_t)TZ_OFFSET_SECONDS;

  uint32_t ss = (uint32_t)(nowEpoch % 60ULL);
  uint32_t mm = (uint32_t)((nowEpoch/60ULL) % 60ULL);
  uint32_t hh = (uint32_t)((nowEpoch/3600ULL) % 24ULL);
  char buf[20];
  snprintf(buf,sizeof(buf),"--/-- %02lu:%02lu:%02lu",
           (unsigned long)hh,(unsigned long)mm,(unsigned long)ss);
  return String(buf);
}

static inline String timeString(){ return timeStringFromEpoch(); }

//================= OLED helpers ======================
static void oledPrint(float bpm, float spo2){
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // Dòng thời gian
  display.setTextSize(1);
  display.setCursor(0,0);
  display.print(timeString());

  // BPM
  display.setCursor(0, 16);
  display.setTextSize(1);
  display.print("BPM:");
  display.setTextSize(2);
  display.setCursor(36, 12);
  if (isnan(bpm)) display.print("--");
  else            display.print((int)roundf(bpm));

  // SpO2
  display.setTextSize(1);
  display.setCursor(0, 40);
  display.print("SpO2:");
  display.setTextSize(2);
  display.setCursor(48, 36);
  if (isnan(spo2)) display.print("--");
  else {
    int v = (int)roundf(spo2);
    display.print(v);
    display.setTextSize(1);
    display.print("%");
  }

  display.display();
}

// Hạn chế tần số refresh OLED
static unsigned long lastOledMs = 0;
static const uint32_t OLED_MIN_INTERVAL_MS = 150;
static void oledMaybeRefresh(){
  unsigned long now = millis();
  if (now - lastOledMs >= OLED_MIN_INTERVAL_MS){
    oledPrint(bpmInstant, spo2PerBeat);
    lastOledMs = now;
  }
}

//================= SpO2 compute ======================
static bool validate_and_compute_spo2(float &spo2Out) {
  if (!haveBeatWindow) return false;

  float dcIR  = (irMax  + irMin ) * 0.5f;
  float dcRED = (redMax + redMin) * 0.5f;
  float acIR  = (irMax  - irMin ) * 0.5f;
  float acRED = (redMax - redMin) * 0.5f;

  if (dcIR < DC_MIN || dcRED < DC_MIN) return false;
  if (dcIR > DC_MAX || dcRED > DC_MAX) return false;
  if (acIR  < MIN_AC_IR || acRED < MIN_AC_RED) return false;

  float piIR  = acIR / dcIR;
  float piRED = acRED / dcRED;
  if (piIR < MIN_PI_IR || piRED < MIN_PI_RED) return false;
  if (dIRmaxAbs > D_IR_MAX) return false;

  float ratioIR  = acIR / dcIR;
  float ratioRED = acRED / dcRED;
  float R = ratioRED / fmaxf(ratioIR, 1e-9f);
  if (R < R_MIN || R > R_MAX) return false;

  float spo2 = CAL_A - CAL_B * R;
  spo2 = fminf(100.0f, fmaxf(70.0f, spo2));
  spo2Out = spo2;
  return true;
}

static inline void reset_beat_window() {
  haveBeatWindow = false;
  irMin = LONG_MAX; irMax = LONG_MIN;
  redMin = LONG_MAX; redMax = LONG_MIN;
  dIRmaxAbs = 0.0f;
  samplesInBeat = 0;
}

static inline void finalize_previous_beat() {
  float spo2;
  if (validate_and_compute_spo2(spo2)) spo2PerBeat = spo2;
  else                                 spo2PerBeat = NAN;
  reset_beat_window();
  oledMaybeRefresh();
}

//================= Autogain ==========================
static void autogain_if_needed(float dcIR, float dcRED){
#if ENABLE_AUTOGAIN
  bool changed = false;
  if (dcIR < DC_TARGET_MIN && LED_IR < LED_MAX)     { LED_IR = min<uint8_t>(LED_MAX, LED_IR + LED_STEP_UP);   changed=true; }
  else if (dcIR > DC_TARGET_MAX && LED_IR > LED_MIN){ LED_IR = max<uint8_t>(LED_MIN, LED_IR - LED_STEP_DOWN); changed=true; }
  if (dcRED < DC_TARGET_MIN && LED_RED < LED_MAX)   { LED_RED = min<uint8_t>(LED_MAX, LED_RED + LED_STEP_UP); changed=true; }
  else if (dcRED > DC_TARGET_MAX && LED_RED > LED_MIN){ LED_RED = max<uint8_t>(LED_MIN, LED_RED - LED_STEP_DOWN); changed=true; }
  if (changed) {
    ppg.setPulseAmplitudeIR(LED_IR);
    ppg.setPulseAmplitudeRed(LED_RED);
  }
#else
  (void)dcIR; (void)dcRED;
#endif
}

//================= PPG sampling (YIELDING) ===========
static inline void sample_ppg_continuous_yielding() {
  static unsigned long lastYield = 0;

  ppg.check();
  while (ppg.available()) {
    long ir  = ppg.getIR();
    long red = ppg.getRed();
    ppg.nextSample();

    bool hasFinger = (ir > FINGER_THRESH);
    unsigned long now = millis();

    if (hasFinger) {
      haveBeatWindow = true;
      if (ir  < irMin ) irMin  = ir;
      if (ir  > irMax ) irMax  = ir;
      if (red < redMin) redMin = red;
      if (red > redMax) redMax = red;

      if (samplesInBeat > 0) {
        float dIR = (float)ir - (float)lastIR;
        if (fabsf(dIR) > dIRmaxAbs) dIRmaxAbs = fabsf(dIR);
      }
      lastIR = ir;
      samplesInBeat++;
    }

    if (hasFinger && checkForBeat(ir)) {
      if (lastBeatMs > 0) {
        unsigned long dt = now - lastBeatMs;
        if (dt >= BEAT_MS_MIN && dt <= BEAT_MS_MAX) {
          float bpm = 60000.0f / (float)dt;
          if (bpm > MIN_BPM && bpm < MAX_BPM) bpmInstant = bpm;
        }
      }
      lastBeatMs = now;

      float dcIR  = (irMax  + irMin ) * 0.5f;
      float dcRED = (redMax + redMin) * 0.5f;
      autogain_if_needed(dcIR, dcRED);

      finalize_previous_beat();
    }

    // 👉 NHƯỜNG CPU CHO LMIC MỖI ~2 ms
    unsigned long t = millis();
    if (t - lastYield >= 2) {
      os_runloop_once();
      lastYield = t;
    }
  }
  oledMaybeRefresh();
}

//================= Uplink ============================
static void schedule_send(uint32_t after_sec, bool add_jitter){
  uint32_t j = (add_jitter && after_sec >= TX_INTERVAL_S) ? (1 + (esp_random() % JITTER_MAX_S)) : 0;
  os_setTimedCallback(&sendjob, os_getTime() + sec_to_osticks(after_sec + j), do_send);
}

static void networkTimeCallback(void *pUserData, int flagSuccess);

void do_send(osjob_t*) {
  if (!g_joined) { schedule_send(PENDING_RETRY_S, false); return; }
  if (LMIC.opmode & OP_TXRXPEND){ schedule_send(PENDING_RETRY_S, false); return; }

  float bpm  = isnan(bpmInstant)  ? 0.0f : bpmInstant;
  float spo2 = isnan(spo2PerBeat) ? 0.0f : spo2PerBeat;

  uint8_t payload[4];
  buildPayload_u16x10(bpm, spo2, payload);
  LMIC_setTxData2(1, payload, sizeof(payload), 0);

  uplinkCount++;
  // ==== DeviceTimeReq: đồng bộ theo chu kỳ định sẵn ====
  if (uplinkCount % RESYNC_EVERY_N_UPLINKS == 0) {
    Serial.println("Requesting network time (periodic)...");
    LMIC_requestNetworkTime(networkTimeCallback, nullptr);
  }

  Serial.printf("TX queued: BPM=%.1f SpO2=%.1f\n", bpm, spo2);
  schedule_send(TX_INTERVAL_S, true);
}

// ==== DeviceTimeReq: callback lấy thời gian mạng ====
static void networkTimeCallback(void *pUserData, int flagSuccess) {
  if (flagSuccess != 1) {
    Serial.println("Network time request failed");
    return;
  }

  lmic_time_reference_t tr;
  if (!LMIC_getNetworkTimeReference(&tr)) {
    Serial.println("LMIC_getNetworkTimeReference failed");
    return;
  }

  // tr.tNetwork là giây theo GPS epoch (1980-01-06, không tính leap seconds)
  const uint32_t GPS_TO_UNIX = 315964800UL; // chênh GPS→UNIX
  const uint8_t  LEAP = 18;                 // leap seconds hiện tại
  uint64_t unixEpoch = (uint64_t)tr.tNetwork + GPS_TO_UNIX - LEAP;

  epoch_base = unixEpoch;
  ms_anchor  = millis();
  haveNetTime = true;

  Serial.printf("NETTIME (MAC) set: epoch=%llu (UTC)\n",
                (unsigned long long)unixEpoch);
  oledPrint(bpmInstant, spo2PerBeat);
}

//================= LMIC events =======================
void onEvent(ev_t ev){
  switch(ev){
    case EV_JOINING:
      Serial.println("EV_JOINING"); g_joined = false; break;

    case EV_JOINED:
      Serial.println("EV_JOINED");
      g_joined = true;

      // ==== DeviceTimeReq: yêu cầu ngay sau khi join ====
      Serial.println("Requesting network time (on join)...");
      LMIC_requestNetworkTime(networkTimeCallback, nullptr);

      schedule_send(1, false);
      break;

    case EV_TXCOMPLETE:
      Serial.println("EV_TXCOMPLETE");
      // Thời gian sẽ được set trong networkTimeCallback nếu DeviceTimeAns về
      break;

    default: break;
  }
}

//================= Setup =============================
void setup(){
  Serial.begin(115200);
  Serial.println("\n[ESP32-C3] MAX30102 → OLED + TTN time (DeviceTimeReq) + Uplink BPM/SpO2");

  // I2C & OLED
  I2C_0.begin(I2C_SDA, I2C_SCL, 400000UL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("❌ OLED not found at 0x3C");
    while(1) delay(1000);
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0,0);
  display.println("Init...");
  display.display();

  // MAX30102
  if (!ppg.begin(I2C_0, I2C_SPEED_FAST, 0x57)) {
    Serial.println("❌ MAX30102 not found! (0x57)");
    display.setCursor(0,10); display.println("MAX30102 ERR");
    display.display();
    while(1) delay(1000);
  }
  ppg.setup();
  ppg.setLEDMode(2);
  ppg.setSampleRate(SAMPLE_RATE_HZ);
  ppg.setPulseWidth(PULSE_WIDTH_US);
  ppg.setPulseAmplitudeIR(LED_IR);
  ppg.setPulseAmplitudeRed(LED_RED);
  ppg.setPulseAmplitudeGreen(0x00);

  // LoRa
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_NSS);
  os_init();
  LMIC_reset();

  // ==== Quan trọng để không lỡ cửa sổ downlink ====
  LMIC_setClockError(MAX_CLOCK_ERROR * 30 / 100);  // 30%
  LMIC_setLinkCheckMode(0);                        // tắt link-check MAC cmd

  LMIC_setAdrMode(0);
  LMIC_setDrTxpow(0, 14);
  LMIC_startJoining();

  oledPrint(NAN, NAN); // khung ban đầu
}

//================= Loop ==============================
void loop(){
  sample_ppg_continuous_yielding();
  os_runloop_once();  // vẫn gọi thêm 1 lần ở cuối vòng
}
