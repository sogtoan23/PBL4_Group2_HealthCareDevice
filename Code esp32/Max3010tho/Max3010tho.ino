// ESP32-C3 + MAX30102: Peak-detect BPM (no smoothing, only noise filtering)
// Requires: SparkFun MAX3010x Sensor Library

#include <Wire.h>
#include "MAX30105.h"

#define I2C_SDA   4
#define I2C_SCL   3
#define I2C_FREQ  400000

MAX30105 sensor;

// ====== Bộ lọc đơn giản ======
// Khử DC: dc += alpha*(x - dc); y = x - dc
// alpha nhỏ → theo chậm, giữ lại dao động nhịp tim (~1–3Hz)
static float dc = 0.0f;
const  float DC_ALPHA = 0.01f;

// Moving average rất ngắn để giảm nhiễu cao tần (không làm mượt đáng kể)
const  int   MA_N = 5;
float maBuf[MA_N];
int   maIdx = 0;
int   maCount = 0;
float maSum = 0.0f;

float movingAverage(float x){
  if (maCount < MA_N) {
    maBuf[maIdx++] = x; if (maIdx == MA_N) maIdx = 0;
    maSum += x; maCount++;
    return maSum / maCount;
  } else {
    maSum -= maBuf[maIdx];
    maBuf[maIdx] = x;
    maSum += x;
    maIdx++; if (maIdx == MA_N) maIdx = 0;
    return maSum / MA_N;
  }
}

// ====== Phát hiện nhịp ======
// Ngưỡng thích nghi theo nhiễu: noise = EMA(|y|), threshold = k*noise
float noiseAbs = 0.0f;
const float NOISE_ALPHA = 0.01f;
const float THR_K = 3.0f;        // nhân hệ số; tăng nếu báo giả

// Khoảng trơ sau khi bắt được nhịp để tránh đếm trùng (ms)
const uint32_t REFRACT_MS = 250; // BPM tối đa ~ 240

// Phạm vi BPM hợp lý để in
const float BPM_MIN = 40.0f;
const float BPM_MAX = 200.0f;

// Trạng thái sườn lên
bool wasBelow = true;
uint32_t lastBeatMs = 0;

// In cho Serial Plotter mỗi mẫu
// Định dạng: IR,Filtered,Threshold,BPM
void printPlot(uint32_t ir, float filtered, float thr, float bpm){
  Serial.print(ir); Serial.print(",");
  Serial.print(filtered, 2); Serial.print(",");
  Serial.print(thr, 2); Serial.print(",");
  Serial.println(bpm, 1);
}

void sensorConfig(){
  // ledBrightness: 0..255 (dòng LED), sampleAverage: 1
  // ledMode: 1 = IR only (đo nhịp), sampleRate: 100 Hz
  // pulseWidth: 411 (18-bit), adcRange: 4096 nA (đủ dải)
  byte ledBrightness = 0x2A; // ~8–9 mA, tùy da tay; tăng/giảm nếu bão hòa hoặc yếu
  byte sampleAverage = 1;
  byte ledMode       = 1;      // IR only
  int  sampleRate    = 100;    // Hz
  int  pulseWidth    = 411;    // us
  int  adcRange      = 4096;   // nA

  sensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);

  // Tuỳ chọn: bật interrupt FIFO đầy/ gần đầy để quét theo lô (không bắt buộc)
  sensor.enableFIFORollover();
}

void setup() {
  Serial.begin(115200);
  delay(50);
  Serial.println("ESP32-C3 + MAX30102 (Peak Detect, no BPM smoothing)");

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_FREQ);

  if (!sensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("Khong tim thay MAX3010x. Kiem tra day noi/I2C!");
    while (1) delay(1000);
  }
  sensorConfig();

  // Hướng dẫn nhanh khi bắt đầu
  Serial.println("# Dat nhe ngon tay len cam bien (che kin anh sang).");
  Serial.println("# Serial Plotter: Tools -> Serial Plotter (115200).");
  Serial.println("# Do thi: IR, Filtered, Threshold, BPM.");

  // Khởi tạo buffer MA
  for (int i = 0; i < MA_N; ++i) maBuf[i] = 0.0f;
}

void loop() {
  // Nạp FIFO của cảm biến vào bộ đệm bên trong thư viện
  sensor.check();

  while (sensor.available()) {
    uint32_t ir = sensor.getFIFOIR(); // chỉ dùng IR cho nhịp tim

    // Bỏ mẫu xấu (IR = 0 hoặc quá nhỏ) khi chưa đặt tay
    if (ir < 5000) { // ngưỡng "không có ngón tay" (tùy module, ánh sáng)
      printPlot(ir, 0, 0, 0);
      sensor.nextSample();
      continue;
    }

    // 1) Khử DC
    dc += DC_ALPHA * ( (float)ir - dc );
    float y = (float)ir - dc;

    // 2) Moving average ngắn (chống nhiễu cao tần một chút)
    float yf = movingAverage(y);

    // 3) Ước lượng nhiễu và ngưỡng
    noiseAbs += NOISE_ALPHA * (fabsf(yf) - noiseAbs);
    float thr = THR_K * noiseAbs;

    // 4) Phát hiện sườn lên vượt ngưỡng + khoảng trơ
    uint32_t now = millis();
    float bpmToPrint = 0.0f;

    if (wasBelow && (yf >= thr) && (now - lastBeatMs > REFRACT_MS)) {
      if (lastBeatMs != 0) {
        float ibi_ms = (float)(now - lastBeatMs); // inter-beat interval
        float bpm = 60000.0f / ibi_ms;
        if (bpm >= BPM_MIN && bpm <= BPM_MAX) {
          // KHÔNG làm mượt BPM: in tức thời
          bpmToPrint = bpm;
        }
      }
      lastBeatMs = now;
      wasBelow = false;
    }
    if (yf < thr) wasBelow = true;

    // In cho Serial Plotter
    printPlot(ir, yf, thr, bpmToPrint);

    sensor.nextSample();
  }
}
