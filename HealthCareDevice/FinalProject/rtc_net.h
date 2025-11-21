#ifndef RTC_H
#define RTC_H

#include <Arduino.h>
#include <stdint.h>

// Cập nhật thời gian (Unix time - giây)
void rtc_sync(uint32_t unixTime);

// Lấy thời gian hiện tại (Unix time - giây). Trả về 0 nếu chưa sync.
uint32_t rtc_now(void);

// Chuỗi thời gian cho OLED, dạng "--/-- HH:MM:SS" (UTC+7)
String rtc_timeString_vn();

#endif
