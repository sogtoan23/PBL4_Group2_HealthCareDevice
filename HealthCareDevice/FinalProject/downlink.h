#ifndef DOWNLINK_H
#define DOWNLINK_H

#include <Arduino.h>
#include <stdint.h>

// Xử lý downlink: fPort=10, payload 4 byte Unix time (big-endian)
void handleDownlink(uint8_t fPort, const uint8_t* payload, uint8_t len);

#endif
