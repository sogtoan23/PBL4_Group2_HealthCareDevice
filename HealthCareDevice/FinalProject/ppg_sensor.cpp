#include "ppg_sensor.h"

#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <math.h>
#include <stdint.h>

#include "lmic_project_config.h"
#include <lmic.h>

//================= MAX30102 (I2C) ====================
#define I2C_SDA  8
#define I2C_SCL  9
MAX30105 ppg;

//================= Cấu hình cảm biến =================
#define SAMPLE_RATE_HZ   100
#define PULSE_WIDTH_US   411
static uint8_t LED_IR  = 0x30;
static uint8_t LED_RED = 0x60;

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
static unsigned long lastBeatMs = 0;
static float g_bpmInstant   = NAN;
static float g_spo2PerBeat  = NAN;

static bool  haveBeatWindow = false;
static long  irMin = LONG_MAX, irMax = LONG_MIN;
static long  redMin = LONG_MAX, redMax = LONG_MIN;
static long  lastIR = 0;
static float dIRmaxAbs = 0.0f;
static uint16_t samplesInBeat = 0;

//================= Getter ============================
float ppg_get_bpm()   { return g_bpmInstant; }
float ppg_get_spo2()  { return g_spo2PerBeat; }

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
  if (validate_and_compute_spo2(spo2)) g_spo2PerBeat = spo2;
  else                                 g_spo2PerBeat = NAN;
  reset_beat_window();
}

//================= Autogain ==========================
static void autogain_if_needed(float dcIR, float dcRED){
#if ENABLE_AUTOGAIN
  bool changed = false;
  if (dcIR < DC_TARGET_MIN && LED_IR < LED_MAX)     { LED_IR = min<uint8_t>(LED_MAX, LED_IR + LED_STEP_UP);   changed=true; }
  else if (dcIR > DC_TARGET_MAX && LED_IR > LED_MIN){ LED_IR = max<uint8_t>(LED_MIN, LED_IR - LED_STEP_DOWN); changed=true; }

  if (dcRED < DC_TARGET_MIN && LED_RED < LED_MAX)   { LED_RED = min<uint8_t>(LED_MAX, LED_RED + LED_STEP_UP);   changed=true; }
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
void ppg_loop_iteration() {
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
          if (bpm > MIN_BPM && bpm < MAX_BPM) g_bpmInstant = bpm;
        }
      }
      lastBeatMs = now;

      float dcIR  = (irMax  + irMin ) * 0.5f;
      float dcRED = (redMax + redMin) * 0.5f;
      autogain_if_needed(dcIR, dcRED);

      finalize_previous_beat();
    }

    // 👉 Nhường CPU cho LMIC mỗi ~2 ms
    unsigned long t = millis();
    if (t - lastYield >= 2) {
      os_runloop_once();
      lastYield = t;
    }
  }
}

//================= Init ==============================
void ppg_init() {
  Wire.begin(I2C_SDA, I2C_SCL, 400000UL);

  if (!ppg.begin(Wire, I2C_SPEED_FAST, 0x57)) {
    Serial.println("❌ MAX30102 not found! (0x57)");
    while(1) delay(1000);
  }
  ppg.setup();
  ppg.setLEDMode(2);
  ppg.setSampleRate(SAMPLE_RATE_HZ);
  ppg.setPulseWidth(PULSE_WIDTH_US);
  ppg.setPulseAmplitudeIR(LED_IR);
  ppg.setPulseAmplitudeRed(LED_RED);
  ppg.setPulseAmplitudeGreen(0x00);
}
