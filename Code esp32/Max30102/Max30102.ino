/*
 * Ghi chú:
 *  - Dùng buffer 100 mẫu (≈ 4s @25 sps nội bộ của lib Maxim)
 *  - Mỗi 1 s (25 mẫu mới) tính lại HR/SpO2, sau đó làm mượt bằng trung bình trượt
 *  - Serial Plotter: 115200 baud, “No line ending”
 *
 * Lựa chọn nhanh (bạn đổi ngay tại mục “THIẾT LẬP NGƯỜI DÙNG”):
 *  - Độ sáng LED: 20–120 (thường 60 ổn); nếu bão hòa, giảm xuống
 *  - sampleAverage: 4–16 (tăng để mượt, nhưng chậm hơn)
 *  - sampleRate: 50–100 đủ; 200–400 khi tay ít rung; 800+ dễ nhiễu chuyển động
 *  - pulseWidth: 411 cho tín hiệu mạnh; 69–118 khi muốn tốc độ cao, ít bão hòa
 *  - adcRange: 2048/4096/8192/16384 — chọn nhỏ khi tín hiệu yếu (tăng độ nhạy), lớn khi dễ bão hòa
 */

#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"

MAX30105 particleSensor;

// ===================== THIẾT LẬP NGƯỜI DÙNG =====================
// (1) Chân I²C cho ESP32-C3 (nếu dùng board khác, sửa/ghi chú bên dưới)
#if defined(ARDUINO_ESP32C3_DEV) || defined(ARDUINO_ESP32C3_DEVKITM_1)
  #define I2C_SDA 4
  #define I2C_SCL 3
  #define I2C_FREQ 400000  // Lựa chọn khác: 100000 (ổn định hơn khi nhiều thiết bị)
#endif

// (2) Cấu hình cảm biến mặc định — có gợi ý lựa chọn khác ngay trong comment
const byte  CFG_LED_BRIGHTNESS = 60;    // Lựa chọn khác: 20–120 (cao quá dễ bão hòa)
const byte  CFG_SAMPLE_AVG     = 4;     // Lựa chọn khác: 8/16 (mượt hơn, chậm hơn)
const byte  CFG_LED_MODE       = 2;     // 1=Red, 2=Red+IR, 3=Red+IR+Green (SpO2: nên 2)
const byte  CFG_SAMPLE_RATE    = 100;   // Lựa chọn khác: 50/200/400 (800+ dễ nhiễu chuyển động)
const int   CFG_PULSE_WIDTH    = 411;   // 69/118/215/411 (411 cho SNR cao)
const int   CFG_ADC_RANGE      = 4096;  // 2048/4096/8192/16384 (tùy mức tín hiệu)

// (3) Làm mượt kết quả
const byte  AVG_SIZE           = 4;     // Lựa chọn khác: 5–8 (mượt hơn, chậm phản ứng)

// ================================================================

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
  // Arduino Uno: RAM hạn chế → dùng 16-bit
  uint16_t irBuffer[100];
  uint16_t redBuffer[100];
  using raw_t = uint16_t;
#else
  // ESP32/STM32/…: dùng 32-bit đầy đủ
  uint32_t irBuffer[100];
  uint32_t redBuffer[100];
  using raw_t = uint32_t;
#endif

// Biến toàn cục cho thuật toán Maxim
const int32_t BUFFER_LEN = 100;   // 100 mẫu ~ 4 giây dữ liệu
int32_t spo2 = 0;                 // SpO2 (%)
int8_t  validSPO2 = 0;            // 1 nếu hợp lệ
int32_t heartRate = 0;            // BPM
int8_t  validHeartRate = 0;       // 1 nếu hợp lệ

// Bộ nhớ để tính trung bình trượt kết quả
int32_t spo2Hist[AVG_SIZE] = {0};
int32_t bpmHist [AVG_SIZE] = {0};
byte    histIdx = 0;
bool    initedWindow = false;

// ============== HÀM TIỆN ÍCH ==============
// Tính trung bình nguyên (đủ dùng cho làm mượt), nhanh gọn
int32_t avgArray(const int32_t *arr, byte n) {
  long sum = 0;
  for (byte i = 0; i < n; i++) sum += arr[i];
  return (int32_t)(sum / n);
}

// In dữ liệu theo format “label:value” để Arduino Serial Plotter hiểu
// CHỈ 2 dòng: Nhịp_tim_TB (BPM) và SpO2_%
// Gợi ý khác: đổi nhãn cho gọn (“BPM_TB”, “SpO2_phan_tram”), hoặc thêm “Valid” nếu cần
void printPlotter(int32_t bpmAvg, int32_t spo2Avg) {
  Serial.print("Bpm:"); Serial.print(bpmAvg);
  Serial.print('\t');
  Serial.print("SpO2_%:");      Serial.println(spo2Avg);
}

// ============== HÀM KHỞI TẠO ==============
// Mục tiêu: khởi tạo Serial, I²C, cảm biến; đọc 100 mẫu đầu tiên rồi tính HR/SpO2
// Lựa chọn khác:
//  - I²C 100 kHz nếu bus dài/nhiều thiết bị
//  - ledBrightness thấp nếu nóng tay/bão hòa
void setup() {
  Serial.begin(115200);

  // Khởi tạo I²C theo board
  #if defined(I2C_SDA) && defined(I2C_SCL)
    Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ); // ESP32-C3
  #else
    Wire.begin();                            // Arduino/STM32: dùng chân mặc định
    // Nếu cần đổi freq: Wire.setClock(400000); // Lựa chọn khác: 100000
  #endif

  // Tìm cảm biến
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println(F("Khong tim thay MAX3010x. Kiem tra day/nguon/I2C!"));
    while (1) delay(10);
  }

  // Cấu hình cảm biến
  particleSensor.setup(
    CFG_LED_BRIGHTNESS,
    CFG_SAMPLE_AVG,
    CFG_LED_MODE,
    CFG_SAMPLE_RATE,
    CFG_PULSE_WIDTH,
    CFG_ADC_RANGE
  );

  // Gợi ý khác:
  // particleSensor.setPulseAmplitudeRed(0x1F);  // tinh chỉnh riêng biên độ LED Red
  // particleSensor.setPulseAmplitudeIR(0x1F);   // tinh chỉnh riêng biên độ LED IR

  Serial.println(F("Bat dau do HR/SpO2 (mo Tools -> Serial Plotter)..."));
}

// ============== HÀM NẠP CỬA SỔ BAN ĐẦU ==============
// Mục tiêu: đọc đủ 100 mẫu (4s) để thuật toán Maxim có dữ liệu khởi đầu ổn định
// Lựa chọn khác:
//  - Có thể bỏ in Plotter giai đoạn đầu để tránh “đồ thị 0” (tại đây mình không in)
void fillInitialWindow() {
  for (int i = 0; i < BUFFER_LEN; i++) {
    while (!particleSensor.available()) particleSensor.check();
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i]  = particleSensor.getIR();
    particleSensor.nextSample();
  }

  // Tính HR/SpO2 lần đầu (thuật toán Maxim làm lọc, phát hiện đỉnh, khử DC, v.v.)
  maxim_heart_rate_and_oxygen_saturation(
      irBuffer, BUFFER_LEN, redBuffer,
      &spo2, &validSPO2, &heartRate, &validHeartRate);

  // Khởi tạo lịch sử để có giá trị trung bình ngay
  for (byte k = 0; k < AVG_SIZE; k++) {
    spo2Hist[k] = (validSPO2 ? spo2 : 0);
    bpmHist[k]  = (validHeartRate ? heartRate : 0);
  }
  histIdx = 0;
  initedWindow = true;

  // Lựa chọn khác:
  //  - Nếu muốn báo “Đang khởi động…”: 
  Serial.println("Khoi tao xong, dang on dinh tin hieu...");
}

// ============== VÒNG LẶP CHÍNH ==============
// Mục tiêu: mỗi lần lấy thêm 25 mẫu (~1s), tính lại HR/SpO2, rồi in NHỊP_TB & SpO2_%
// Lựa chọn khác:
//  - Tăng AVG_SIZE để đồ thị mượt hơn (đổi ở trên)
//  - Giảm CFG_SAMPLE_RATE nếu nhiễu chuyển động nhiều
void loop() {
  if (!initedWindow) { fillInitialWindow(); return; }

  // Dịch 75 mẫu cũ xuống (giữ 75 mẫu cuối)
  for (int i = 25; i < BUFFER_LEN; i++) {
    redBuffer[i - 25] = redBuffer[i];
    irBuffer[i - 25]  = irBuffer[i];
  }

  // Lấy 25 mẫu mới (≈ 1 giây dữ liệu)
  for (int i = BUFFER_LEN - 25; i < BUFFER_LEN; i++) {
    while (!particleSensor.available()) particleSensor.check();
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i]  = particleSensor.getIR();
    particleSensor.nextSample();
  }

  // Tính lại HR/SpO2 trên cửa sổ 100 mẫu
  maxim_heart_rate_and_oxygen_saturation(
      irBuffer, BUFFER_LEN, redBuffer,
      &spo2, &validSPO2, &heartRate, &validHeartRate);

  // Cập nhật lịch sử cho trung bình trượt
  if (validHeartRate) bpmHist[histIdx] = heartRate;
  if (validSPO2)      spo2Hist[histIdx] = spo2;
  histIdx = (histIdx + 1) % AVG_SIZE;

  // Tính giá trị trung bình để vẽ cho mượt
  int32_t bpmAvg  = avgArray(bpmHist, AVG_SIZE);
  int32_t spo2Avg = avgArray(spo2Hist, AVG_SIZE);

  // CHỈ IN 2 KÊNH: BPM & SpO2 (không in IR/RED thô)
  printPlotter(bpmAvg, spo2Avg);

  // Lựa chọn khác:
  //  - In thêm “độ tin cậy”: Serial.printf("\tHRvalid:%d\tSPO2valid:%d\n", validHeartRate, validSPO2);
  //  - Throttle tần số in (nếu quá tải): delay(50);  // nhưng sẽ trễ đồ thị
}

/*
 * ===================== MẸO HIỆU CHỈNH NHANH =====================
 * - “Số nhảy”: tăng AVG_SIZE hoặc CFG_SAMPLE_AVG, giảm CFG_SAMPLE_RATE
 * - “Bão hòa” (giá trị IR/RED rất lớn): giảm CFG_LED_BRIGHTNESS hoặc tăng CFG_ADC_RANGE
 * - “Tín hiệu yếu”: tăng CFG_LED_BRIGHTNESS hoặc giảm CFG_ADC_RANGE (2048/4096)
 * - “Nhiễu I²C”: dùng 100 kHz, thêm pull-up 4.7–10 kΩ lên 3V3, dây ngắn, tránh vòng mass
 * - ESP32-C3: nếu board map SDA/SCL khác, sửa I2C_SDA/I2C_SCL cho đúng
 */
