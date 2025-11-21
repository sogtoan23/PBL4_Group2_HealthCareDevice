#ifndef LORA_TTN_H
#define LORA_TTN_H

#include <Arduino.h>

// Khởi tạo SPI + LMIC + join TTN + DeviceTimeReq on join
void lora_init();

// Gọi trong loop để LMIC xử lý event (ngoài phần yield trong PPG)
void lora_process();

#endif
