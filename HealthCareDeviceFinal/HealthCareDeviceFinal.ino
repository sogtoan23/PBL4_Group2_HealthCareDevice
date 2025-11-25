#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Wire.h>
#include <time.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "MAX30105.h" // Yêu cầu thư viện SparkFun MAX3010x

// ===================== CẤU HÌNH TTN (OTAA) =====================
// LƯU Ý QUAN TRỌNG: Các Key phải ở định dạng LSB (Little Endian)
// Copy từ TTN Console -> Nhấn nút "<>" để lấy dạng mảng byte

static const u1_t PROGMEM APPEUI[8]  = { 0xCD, 0xAB, 0x00, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
static const u1_t PROGMEM DEVEUI[8]  = { 0xE6, 0x4C, 0x00, 0xD8, 0x7E, 0xD5, 0xB3, 0x70 };
static const u1_t PROGMEM APPKEY[16] = {
    0x14, 0x67, 0xDB, 0x5E, 0xC9, 0x4C, 0x46, 0xA6,
    0x21, 0x5F, 0x04, 0xB4, 0x53, 0x44, 0x06, 0x92
};

void os_getArtEui(u1_t* buf) { memcpy_P(buf, APPEUI, 8); }
void os_getDevEui(u1_t* buf) { memcpy_P(buf, DEVEUI, 8); }
void os_getDevKey(u1_t* buf) { memcpy_P(buf, APPKEY, 16); }

// ================== PINMAP (ESP32-C3 ↔ SX1276) =====================
#define PIN_SCK   4
#define PIN_MISO  5
#define PIN_MOSI  6
#define PIN_NSS   7
#define PIN_RST   3
#define PIN_DIO0  2
#define PIN_DIO1  1

const lmic_pinmap lmic_pins = {
    .nss  = PIN_NSS,
    .rxtx = LMIC_UNUSED_PIN,
    .rst  = PIN_RST,
    .dio  = { PIN_DIO0, PIN_DIO1, LMIC_UNUSED_PIN },
};

// ================== CẤU HÌNH OLED & SENSOR =====================
#define OLED_SDA   8
#define OLED_SCL   9
#define OLED_ADDR  0x3C
Adafruit_SSD1306 display(128, 64, &Wire, -1);
MAX30105 ppg;

// ================== BIẾN TOÀN CỤC =====================
float g_bpm  = 0.0f;
float g_spo2 = 0.0f;

// --- CỜ KIỂM SOÁT CẢM BIẾN ---
// true = Đang ưu tiên LoRa (dừng cảm biến)
// false = Cảm biến chạy bình thường
volatile bool isSensorPaused = false; 

// --- Biến thuật toán xử lý tín hiệu tim ---
static uint32_t lastBeatMs = 0;
static float    bpmEMA     = 0.0f;
static float    dcIR = 0.0f, dcRed = 0.0f;
static float    acIR = 0.0f, acRed = 0.0f;
static float    thresh = 0.0f;
static bool     wasHigh = false;

// --- Biến thời gian ---
uint32_t g_unixBase   = 0;  // Thời gian gốc (Epoch)
uint32_t g_millisBase = 0;  // Millis lúc nhận sync

uint32_t getCurrentUnix() {
    if (g_unixBase == 0) return 0;
    return g_unixBase + (millis() - g_millisBase) / 1000;
}

// ================== UPLINK TIMING =========================
static osjob_t sendjob;
const unsigned TX_INTERVAL_S = 30; // Chu kỳ gửi 30s
volatile bool g_joined = false;

// ================== XỬ LÝ CẢM BIẾN (MAX30102) =======================
void updatePpgSensor() {
    // [SMART SYNC LOGIC]
    // Nếu cờ này bật -> CPU cần rảnh để nhận Downlink -> Return ngay
    if (isSensorPaused) {
        return; 
    }

    ppg.check(); // Kiểm tra dữ liệu mới

    while (ppg.available()) {
        uint32_t ir  = ppg.getIR();
        uint32_t red = ppg.getRed();

        // --- Bộ lọc DC / AC ---
        const float alphaDC = 0.99f;
        const float alphaAC = 0.90f;

        if (dcIR == 0.0f) { dcIR = ir; dcRed = red; }

        dcIR  = alphaDC * dcIR  + (1.0f - alphaDC) * (float)ir;
        dcRed = alphaDC * dcRed + (1.0f - alphaDC) * (float)red;

        float vIR  = (float)ir  - dcIR;
        // float vRed = (float)red - dcRed;

        acIR  = alphaAC * acIR  + (1.0f - alphaAC) * fabsf(vIR);
        acRed = alphaAC * acRed + (1.0f - alphaAC) * fabsf((float)red - dcRed);

        // --- Thuật toán phát hiện đỉnh (Peak Detection) ---
        float dynThresh = 0.65f * acIR;
        thresh = 0.8f * thresh + 0.2f * dynThresh; 

        bool isHigh = (vIR > thresh);
        uint32_t nowMs = millis();

        if (isHigh && !wasHigh) { // Sườn lên
            if (lastBeatMs > 0) {
                uint32_t delta = nowMs - lastBeatMs;
                if (delta > 300 && delta < 2000) { // 30-200 BPM
                    float bpmInstant = 60000.0f / (float)delta;
                    if (bpmEMA == 0.0f) bpmEMA = bpmInstant;
                    else bpmEMA = 0.8f * bpmEMA + 0.2f * bpmInstant;
                    g_bpm = bpmEMA;
                }
            }
            lastBeatMs = nowMs;
        }
        wasHigh = isHigh;

        // --- Tính SpO2 (Ratio of Ratios) ---
        if (dcIR > 1000.0f && dcRed > 1000.0f && acIR > 1.0f && acRed > 1.0f) {
            float R = ( (acRed/dcRed) / (acIR/dcIR) ); 
            float spo2 = 110.0f - 25.0f * R;          
            if (spo2 < 0.0f) spo2 = 0.0f;
            if (spo2 > 100.0f) spo2 = 100.0f;

            if (g_spo2 == 0.0f) g_spo2 = spo2;
            else g_spo2 = 0.9f * g_spo2 + 0.1f * spo2;
        }

        ppg.nextSample(); 
    }
    
    if (g_spo2 == 0.0f) g_spo2 = 99.0f; // Giá trị mặc định
}

// ================== HIỂN THỊ OLED ==========================
void updateDisplay() {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);

    display.setTextSize(2);
    display.setCursor(0, 0);
    display.print("BPM: "); 
    
    // Nếu đang tạm dừng cảm biến, hiển thị chữ SYNC nhấp nháy
    if (isSensorPaused) {
        display.print("SYNC"); 
    } else {
        display.println((int)g_bpm);
    }

    display.setCursor(0, 24);
    display.print("SpO2: "); display.println((int)g_spo2);

    display.setTextSize(1);
    display.setCursor(0, 52);

    uint32_t nowUnix = getCurrentUnix();
    if (nowUnix == 0) {
        display.print("Waiting Time...");
    } else {
        time_t t = (time_t)(nowUnix + 7 * 3600); // Múi giờ VN (UTC+7)
        struct tm *tm_info = gmtime(&t);
        char buf[32]; // Buffer đủ chứa "DD/MM/YYYY HH:MM:SS"
        // Định dạng: "DD/MM/YYYY HH:MM:SS"
        snprintf(buf, sizeof(buf), "%02d/%02d/%04d %02d:%02d:%02d",
                 tm_info->tm_mday, tm_info->tm_mon + 1, tm_info->tm_year + 1900,
                 tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec);
        display.print(buf);
    }
    display.display();
}

// ================== ĐÓNG GÓI DỮ LIỆU ========================
void buildPayload_u16x10(float bpm, float spo2, uint8_t *out) {
    auto enc = [](float v) -> uint16_t {
        if (isnan(v)) v = 0;
        if (v > 6553.5f) v = 6553.5f;
        return (uint16_t)lroundf(v * 10.0f);
    };
    uint16_t b = enc(bpm);
    uint16_t s = enc(spo2);
    out[0] = (b >> 8) & 0xFF; out[1] = b & 0xFF;
    out[2] = (s >> 8) & 0xFF; out[3] = s & 0xFF;
}

// ================== GỬI LORA & QUẢN LÝ CẢM BIẾN ==========================
static void do_send(osjob_t*);

static void schedule_send(uint32_t sec) {
    os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(sec), do_send);
}

static void do_send(osjob_t*) {
    if (!g_joined) {
        schedule_send(5);
        return;
    }
    if (LMIC.opmode & OP_TXRXPEND) {
        schedule_send(5);
        return;
    }

    // Đọc cảm biến lần cuối trước khi gửi
    updatePpgSensor();

    // ================= [QUAN TRỌNG: LOGIC SMART PAUSE] =================
    // Nếu chưa có thời gian (g_unixBase == 0):
    // -> Bật cờ dừng cảm biến để ưu tiên CPU nhận Downlink Time Sync
    // Nếu đã có thời gian:
    // -> Không bật cờ, cảm biến vẫn chạy nền bình thường.
    
    if (g_unixBase == 0) {
        isSensorPaused = true;
        Serial.println(F("TX: No Time -> Pausing Sensor to wait for Downlink..."));
    } else {
        isSensorPaused = false;
        Serial.println(F("TX: Time Synced -> Sensor continues running."));
    }
    // ===================================================================

    uint8_t payload[5];
    buildPayload_u16x10(g_bpm, g_spo2, payload);
    payload[4] = (g_unixBase == 0) ? 1 : 0; // Flag=1 báo server cần gửi giờ về

    // Gửi lên Port 1
    LMIC_setTxData2(1, payload, sizeof(payload), 0);
}

// ================== XỬ LÝ SỰ KIỆN LORA ==================
void onEvent(ev_t ev) {
    switch (ev) {
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
            
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            g_joined = true;
            LMIC_setLinkCheckMode(0);
            LMIC_setAdrMode(0); // Tắt ADR để ổn định
            
            // Join xong -> Reset cờ để cảm biến chạy
            isSensorPaused = false;
            schedule_send(5);
            break;
            
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE"));
            
            // [QUAN TRỌNG] Gửi/Nhận xong -> Tắt cờ dừng cảm biến ngay lập tức
            // Để đảm bảo dù có nhận được giờ hay không, cảm biến vẫn chạy lại
            isSensorPaused = false;
            
            // Xả bộ đệm cảm biến để xóa dữ liệu cũ
            ppg.clearFIFO(); 

            // --- KIỂM TRA DOWNLINK ---
            if (LMIC.dataLen > 0) {
                uint8_t fport = LMIC.frame[LMIC.dataBeg - 1];
                const uint8_t* buf = LMIC.frame + LMIC.dataBeg;
                uint8_t len = LMIC.dataLen;

                Serial.printf("Downlink Port: %d, Len: %d\n", fport, len);

                // Port 10 chứa 4 byte Time Stamp
                if (fport == 10 && len >= 4) {
                    uint32_t t = (uint32_t)buf[0] << 24 | 
                                 (uint32_t)buf[1] << 16 | 
                                 (uint32_t)buf[2] << 8  | 
                                 (uint32_t)buf[3];
                    g_unixBase = t;
                    g_millisBase = millis();
                    Serial.printf(">>> TIME SYNC SUCCESS: %u\n", t);
                }
            }
            // -------------------------
            
            schedule_send(TX_INTERVAL_S);
            break;
            
        default:
            break;
    }
}

// ================== SETUP ==========================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println(F("ESP32 Health Monitor - Smart Sync"));

    // 1. Khởi động OLED
    Wire.begin(OLED_SDA, OLED_SCL);
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        Serial.println(F("OLED Fail"));
        while (1);
    }
    display.clearDisplay();
    display.display();

    // 2. Khởi động MAX30102
    if (!ppg.begin(Wire, I2C_SPEED_FAST)) {
        Serial.println(F("MAX30102 not found"));
        display.setCursor(0,0);
        display.print("Sensor Error!");
        display.display();
        while (1);
    }
    // Cấu hình: Độ sáng cao, Lấy mẫu trung bình, Mode Red+IR, Tốc độ 400Hz
    ppg.setup(0x3F, 4, 2, 400, 411, 16384); 
    ppg.setPulseAmplitudeGreen(0); // Tắt led xanh

    // 3. Khởi động LoRa
    SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_NSS);
    os_init();
    LMIC_reset();
    
    // Tăng dung sai đồng hồ (quan trọng cho ESP32)
    LMIC_setClockError(MAX_CLOCK_ERROR * 20 / 100);
    
    // RX Delay chuẩn 5s (TTN v3)
    LMIC.rxDelay = 5;

    // Dùng SF10 để truyền nhanh hơn SF12
    LMIC_setDrTxpow(DR_SF10, 14); 

    LMIC_startJoining();
}

// ================== LOOP ==========================
void loop() {
    os_runloop_once(); // Xử lý nền LoRa

    // Hàm này tự kiểm tra cờ isSensorPaused
    updatePpgSensor();

    static uint32_t lastDisp = 0;
    if (millis() - lastDisp > 500) {
        lastDisp = millis();
        updateDisplay();
    }
}