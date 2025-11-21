#ifndef PPG_SENSOR_H
#define PPG_SENSOR_H

#include <Arduino.h>

// Khởi tạo I2C + MAX30102
void ppg_init();

// Lấy mẫu PPG liên tục, phát hiện beat, tính BPM & SpO2
// Bên trong có nhường CPU cho LMIC bằng os_runloop_once().
void ppg_loop_iteration();

// Getter cho giá trị tức thời (per-beat)
float ppg_get_bpm();
float ppg_get_spo2();

#endif
