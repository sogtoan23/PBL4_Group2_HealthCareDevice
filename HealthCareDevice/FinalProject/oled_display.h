#ifndef OLED_DISPLAY_H
#define OLED_DISPLAY_H

#include <Arduino.h>

// Khởi tạo OLED SSD1306
void oled_init();

// Vẽ ngay lập tức (không hạn chế tần số)
void oled_force(float bpm, float spo2);

// Vẽ nhưng giới hạn tần số (debounce refresh)
void oled_maybe(float bpm, float spo2);

#endif
