#include <Wire.h>

#define SENSOR_ADDR 0x57   // địa chỉ I2C chung cho cả MAX30100 & MAX30102
#define REG_PART_ID 0xFF   // Thanh ghi Part ID

void setup() {
  Serial.begin(115200);
  delay(200);

  // Arduino UNO: SDA=A4, SCL=A5
  // ESP32-C3: thay bằng Wire.begin(SDA, SCL);
  Wire.begin();
  Wire.setClock(100000);

  Serial.println("=== Kiem tra MAX3010x ===");

  Wire.beginTransmission(SENSOR_ADDR);
  Wire.write(REG_PART_ID);
  if (Wire.endTransmission(false) == 0 && Wire.requestFrom(SENSOR_ADDR, 1) == 1) {
    byte part = Wire.read();
    Serial.print("PART_ID = 0x"); Serial.println(part, HEX);

    if (part == 0x15) {
      Serial.println("=> Day la MAX30102");
    } else if (part == 0x11) {
      Serial.println("=> Day la MAX30100");
    } else {
      Serial.println("=> Khong phai MAX30100/30102 hoac IC la fake");
    }
  } else {
    Serial.println("Khong thay thiet bi @0x57");
  }
}

void loop() {}
