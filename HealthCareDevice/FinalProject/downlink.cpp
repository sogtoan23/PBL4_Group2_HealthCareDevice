#include <Arduino.h>
#include "downlink.h"
#include "rtc_net.h"

void handleDownlink(uint8_t fPort, const uint8_t* payload, uint8_t len) {

    // Chỉ xử lý fPort = 10 và payload 4 bytes (Unix time big-endian)
    if (fPort == 10 && len == 4) {

        uint32_t t = 0;
        t |= ((uint32_t)payload[0] << 24);
        t |= ((uint32_t)payload[1] << 16);
        t |= ((uint32_t)payload[2] << 8);
        t |= ((uint32_t)payload[3]);

        rtc_sync(t);

        Serial.print("RTC synced! Unix time = ");
        Serial.println(rtc_now());
    }
}
