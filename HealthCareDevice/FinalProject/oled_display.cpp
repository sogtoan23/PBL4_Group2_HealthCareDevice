#include "oled_display.h"

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "rtc_net.h"
#include <math.h>

// Dùng chung bus I2C (có thể đã begin ở ppg_init)
#define I2C_SDA  8
#define I2C_SCL  9

#define OLED_ADDR 0x3C
#define OLED_W    128
#define OLED_H    64

static Adafruit_SSD1306 display(OLED_W, OLED_H, &Wire, -1);

static unsigned long lastOledMs = 0;
static const uint32_t OLED_MIN_INTERVAL_MS = 150;

static void oled_draw(float bpm, float spo2){
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // Dòng thời gian
  display.setTextSize(1);
  display.setCursor(0,0);
  display.print(rtc_timeString_vn());

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

void oled_init() {
  // Đảm bảo I2C đã begin; nếu chưa thì:
  Wire.begin(I2C_SDA, I2C_SCL, 400000UL);

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
}

void oled_force(float bpm, float spo2) {
  oled_draw(bpm, spo2);
  lastOledMs = millis();
}

void oled_maybe(float bpm, float spo2) {
  unsigned long now = millis();
  if (now - lastOledMs >= OLED_MIN_INTERVAL_MS){
    oled_draw(bpm, spo2);
    lastOledMs = now;
  }
}
