#include <Wire.h>
#include "MAX30100_PulseOximeter.h"

//////////////// CẤU HÌNH ESP32-C3 I2C //////////////////
#define I2C_SDA 4
#define I2C_SCL 3
#define I2C_FREQ 100000
// Cấu hình current của LED
#define LED_CURRENT MAX30100_LED_CURR_14_2MA  // 7_6MA / 11MA / 14_2MA / 17_4MA / 20_8MA
/////////////////////////////////////////////////////////

// Tham số để lọc dữ liệu nhịp tim
// In mỗi 200 ms (5 Hz) cho Plotter
#define PRINT_INTERVAL_MS 200
#define WARMUP_MS 4000

// Bộ lọc EMA
#define EMA_ALPHA_HR 0.30f
#define EMA_ALPHA_SPO2 0.30f

// Miền hợp lý
#define HR_MIN 40.0f
#define HR_MAX 200.0f
#define SPO2_MIN 70.0f
#define SPO2_MAX 100.5f
/////////////////////////////////////////////////////////
// Tạo các giá trị
PulseOximeter pox;
///////////////////////////////////////////////////////
// Tạo biến thời gian
uint32_t lastTimePrint = 0;
uint32_t startTimePrint = 0;
uint32_t now = 0;
//////////////////////////////////////////////////////
// Tạo các giá trị của HR và SPO2
float HR = 0, SPO2 = 0;              // Dữ liệu chưa xử lí
float HRema = 0.0f, SPO2ema = 0.0f;  // Dữ liệu  đã lọc
float HR_final = 0, SPO2_final = 0;  //Dữ liệu cuối
///////////////////////////////////////////////////////
// ====== Mảng HR để lấy trung bình ======
const byte RATE_SIZE = 4;    // 4 phần tử
float rates[RATE_SIZE] = {0};
byte rateSpot = 0;
float HR_avg = 0; 
// ====== Mảng SP02 để lấy trung bình ======
const byte SPO2_SIZE = 4;
float spo2Arr[SPO2_SIZE] = {0};
byte spo2Spot = 0;
float SPO2_avg = 0;

///////////////////////////////////////////////////////////////////
// Tiện ích
static inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}
////////////////////////////////////////////////////////////////////
// Tạo hàm kiểm tra cảm biến
bool Check_i2c_Heart() {
  Wire.beginTransmission(0x57);
  return (Wire.endTransmission() == 0);
}
void setup() {
  Serial.begin(115200);
  delay(200);
  // Khởi tạo i2c cho thiết bị
  Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ);
  // Kiểm tra cảm biến có hoạt động hay không
  if (!Check_i2c_Heart()) {
    Serial.println(F("Cam bien khong hoat dong.Kiem tra lai ket noi thiet bi"));
  }
  // Khởi tạo cảm biến
  Serial.print(F("Dang khoi dong..."));
  if (!pox.begin()) {  // dùng cùng đối tượng Wire vừa init
    Serial.println(F("Loi cam bien nhip tim"));
    while (1) delay(10);
  }
  Serial.println(F("Moi ban dat tay vao cam bien"));
   Serial.println(F("HR\tSpO2"));
  // Tạo dòng cho cảm biến
  pox.setIRLedCurrent(LED_CURRENT);
  // In lên plotter
 
   startTimePrint = millis();
}

void loop() {
  // BẮT BUỘC gọi thường xuyên để thư viện cập nhật thuật toán
  pox.update();

  now = millis();
  if (now - lastTimePrint >= PRINT_INTERVAL_MS) {
    HR = pox.getHeartRate();  // bpm (NaN/0 nếu chưa có)
    SPO2 = pox.getSpO2();     // %

    //Kiểm tra tránh giữ liệu giả
   if (HR < HR_MIN || HR > HR_MAX) {
  HR = 0;
}
 
if (SPO2 < SPO2_MIN || SPO2 > SPO2_MAX) {
  SPO2 = 0;
}

    // Kiểm tra hợp lý rồi EMA
    if (HR > HR_MIN && HR < HR_MAX) {
      HRema = (1.0f - EMA_ALPHA_HR) * HRema + EMA_ALPHA_HR * HR;
    }
    if (SPO2 > SPO2_MIN && SPO2 <= SPO2_MAX) {
      SPO2ema = (1.0f - EMA_ALPHA_SPO2) * SPO2ema + EMA_ALPHA_SPO2 * SPO2;
    }

    bool warmed = (now - startTimePrint) > WARMUP_MS;
    if (warmed) {
    HR_final = warmed ? clampf(HRema, HR_MIN, HR_MAX) : 0.0f;
    SPO2_final = warmed ? clampf(SPO2ema, SPO2_MIN, SPO2_MAX) : 0.0f;
    // ======= Cập nhật mảng HR để lấy trung bình =======
    if (HR_final > HR_MIN && HR_final < HR_MAX) {
      rates[rateSpot++] = HR_final;
      if (rateSpot >= RATE_SIZE) rateSpot = 0;

      float sum = 0;
      for (byte i = 0; i < RATE_SIZE; i++) sum += rates[i];
      HR_avg = sum / RATE_SIZE;
    }
    // ==================================================
    if (SPO2_final > SPO2_MIN && SPO2_final < SPO2_MAX) {
  spo2Arr[spo2Spot++] = SPO2_final;
  if (spo2Spot >= SPO2_SIZE) spo2Spot = 0;

  float sum = 0;
  for (byte i = 0; i < SPO2_SIZE; i++) sum += spo2Arr[i];
  SPO2_avg = sum / SPO2_SIZE;
}


    // In 2 cột cho Serial Plotter
    Serial.print(HR_avg, 1);
    Serial.print('\t');
    Serial.println(SPO2_avg, 1);
    }
    else {
      Serial.println("Cho doi trong 5s");
    }  
    
    

    lastTimePrint = now;
  }
}
