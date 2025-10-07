#include <Wire.h>
#include "MAX30100_PulseOximeter.h"

PulseOximeter pox;

uint32_t lastPrint = 0;     // để in đều tay cho Serial Plotter
float hr_s = 0, spo2_s = 0; // giá trị đã làm mượt nhẹ

// (không bắt buộc) nháy LED khi phát hiện nhịp
void onBeatDetected() {
  // có thể chớp LED_BUILTIN ở đây nếu muốn
}

void setup() {
  Serial.begin(115200);
  Wire.begin();                   // UNO: SDA=A4, SCL=A5

  Serial.println(F("Khoi tao MAX30100 (HR + SpO2)..."));
  if (!pox.begin()) {
    Serial.println(F("Khong tim thay MAX30100!"));
    while (1) {}
  }

  // Dòng LED: tăng/giảm tùy điều kiện ánh sáng & ngón tay
  pox.setIRLedCurrent(MAX30100_LED_CURR_14_2MA);  // 7.6/11/14.2/17.4/20.8 mA
  pox.setOnBeatDetectedCallback(onBeatDetected);

  // Header cho Serial Plotter (2 cột: HR và SpO2)
  Serial.println(F("HR\tSpO2"));
}

void loop() {
  // BẮT BUỘC gọi thường xuyên để thư viện cập nhật thuật toán
  pox.update();

  // In đều 5 lần/giây để Plotter vẽ mượt
  if (millis() - lastPrint > 200) {
    float hr   = pox.getHeartRate();  // bpm
    float spo2 = pox.getSpO2();       // %

    // lọc mượt nhẹ (EMA)
    if (hr > 0 && hr < 220)   hr_s   = 0.7f*hr_s   + 0.3f*hr;   // tránh NaN/0
    if (spo2 > 0 && spo2 <= 100) spo2_s = 0.7f*spo2_s + 0.3f*spo2;

    // In số thuần cho Serial Plotter (tab giữa các cột)
    Serial.print(isnan(hr_s)   ? 0 : hr_s);   Serial.print('\t');
    Serial.println(isnan(spo2_s)? 0 : spo2_s);

    lastPrint = millis();
  }
}
