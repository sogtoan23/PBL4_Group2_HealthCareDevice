#include "rtc_net.h"

// RTC mềm
static uint32_t baseUnixTime = 0;
static uint32_t baseMillis   = 0;

void rtc_sync(uint32_t unixTime) {
    baseUnixTime = unixTime;
    baseMillis   = millis();
}

uint32_t rtc_now(void) {
    if (baseUnixTime == 0) return 0;  // chưa sync
    uint32_t elapsed = (millis() - baseMillis) / 1000;
    return baseUnixTime + elapsed;
}

String rtc_timeString_vn() {
    uint32_t nowUnix = rtc_now();

    // Nếu chưa sync: dùng millis() làm đồng hồ tạm
    if (nowUnix == 0) {
        unsigned long s = millis() / 1000;
        uint16_t hh = (s / 3600) % 24;
        uint16_t mm = (s / 60) % 60;
        uint16_t ss = s % 60;
        char buf[20];
        snprintf(buf, sizeof(buf), "--/-- %02u:%02u:%02u", hh, mm, ss);
        return String(buf);
    }

    // UTC+7
    int64_t local = (int64_t)nowUnix + 7 * 3600;
    if (local < 0) local = 0;

    uint32_t ss = (uint32_t)(local % 60);
    uint32_t mm = (uint32_t)((local / 60) % 60);
    uint32_t hh = (uint32_t)((local / 3600) % 24);

    char buf[20];
    snprintf(buf, sizeof(buf), "--/-- %02lu:%02lu:%02lu",
             (unsigned long)hh, (unsigned long)mm, (unsigned long)ss);
    return String(buf);
}
