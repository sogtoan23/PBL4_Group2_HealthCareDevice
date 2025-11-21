#ifndef _LMIC_PROJECT_CONFIG_H_
#define _LMIC_PROJECT_CONFIG_H_

// ===== Region: AS923 Group-1 (JP/PH/KR) =====
 #define CFG_as923 1

// KHÔNG bật thêm region khác
//#define CFG_as923jp 1
// #define CFG_us915 1
// #define CFG_au915 1
// #define CFG_eu868 1
// #define CFG_in866 1

#define CFG_sx1276_radio 1
#define LMIC_PRINTF_TO Serial
#define LMIC_DEBUG_LEVEL 2

// ⚙️ Chọn chuẩn PHY không phải Rev-B
#define LMIC_LORAWAN_SPEC_VERSION   LMIC_LORAWAN_SPEC_VERSION_1_0_2
// KHÔNG thêm dòng LMIC_LORAWAN_SPEC_VERSION_1_0_2_REV_B ở đây

// Gỡ lỗi / tiết kiệm RAM
#define DISABLE_PING
#define DISABLE_BEACONS
#define LMIC_ENABLE_DeviceTimeReq 1

#endif
