#include <Wire.h>
#include "MAX30100.h"   // <- thư viện lớp thấp

// ESP32-C3 I2C
#define I2C_SDA   4
#define I2C_SCL   3
#define I2C_FREQ  100000

MAX30100 sensor;

static inline bool i2cFound(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ);

  if (!i2cFound(0x57)) {
    Serial.println(F("Khong thay MAX30100 (0x57). Kiem tra day noi!"));
    // vẫn tiếp tục thử để in lỗi từ begin()
  }

  // Khởi tạo cảm biến
  if (!sensor.begin()) {
    Serial.println(F("MAX30100 begin() FAILED!"));
    while (1) delay(10);
  }

  // Cấu hình cơ bản cho lấy RAW (giữ mức an toàn cho ngón tay)
  // Lưu ý: tên enum có thể hơi khác tuỳ phiên bản thư viện,
  // nếu IDE báo lỗi tên, mở file MAX30100.h để chọn đúng tên tương ứng.

  sensor.setMode( MAX30100_MODE_SPO2_HR);                 // chế độ SpO2 (đọc cả IR & RED)
  sensor.setLedsCurrent(MAX30100_LED_CURR_14_2MA,     // RED current
                        MAX30100_LED_CURR_14_2MA);    // IR current
  sensor.setLedsPulseWidth(MAX30100_SPC_PW_1600US_16BITS);   // xung rộng -> biên độ tốt hơn
  sensor.setSamplingRate(MAX30100_SAMPRATE_100HZ);    // tốc độ lấy mẫu

  // Header cho Serial Plotter (nếu IDE 2.x không nhận header, vẫn OK cho Monitor)
  Serial.println(F("IR\tRED"));
}

void loop() {
  // BẮT BUỘC gọi thường xuyên để thư viện đẩy FIFO ra bộ đệm
  sensor.update();

  // Lấy tất cả mẫu có trong FIFO và in ra
  uint16_t ir, red;

  // Tùy phiên bản thư viện, một trong hai hàm dưới đây có sẵn:
  // 1) getRawValues(&ir,&red) trả về true nếu có mẫu
  // 2) available()/getIR()/getRed()/nextSample() theo kiểu vòng đệm

  // --- Cách 1: dùng getRawValues (phổ biến) ---
  while (sensor.getRawValues(&ir, &red)) {
    Serial.print(ir);
    Serial.print('\t');
    Serial.println(red);
  }

  // --- Nếu IDE báo không có getRawValues, hãy dùng Cách 2: ---
  // while (sensor.available()) {
  //   ir  = sensor.getIR();
  //   red = sensor.getRed();
  //   Serial.print(ir);
  //   Serial.print('\t');
  //   Serial.println(red);
  //   sensor.nextSample();
  // }
}
