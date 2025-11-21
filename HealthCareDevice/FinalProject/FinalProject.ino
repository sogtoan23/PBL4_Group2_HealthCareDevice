#include <Arduino.h>

#include "rtc_net.h"
#include "downlink.h"
#include "ppg_sensor.h"
#include "oled_display.h"
#include "lora_ttn.h"

void setup() {
  Serial.begin(115200);
  Serial.println("\n[ESP32-C3] MAX30102 → OLED + TTN RTC (downlink fPort=10) + Uplink BPM/SpO2");

  ppg_init();      // MAX30102 + I2C
  oled_init();     // OLED 0.96"
  lora_init();     // LoRaWAN + join TTN (trong đó sẽ gọi handleDownlink() khi có downlink)
}

void loop() {
  // Lấy mẫu PPG liên tục + tính BPM/SpO2 (bên trong có nhường CPU cho LMIC)
  ppg_loop_iteration();

  // Cho LMIC xử lý thêm (timer, event, TX/RX, downlink -> handleDownlink -> rtc_sync)
  lora_process();

  // Cập nhật OLED (giới hạn tần số refresh), dùng BPM/SpO2 hiện tại
  oled_maybe(ppg_get_bpm(), ppg_get_spo2());
}
