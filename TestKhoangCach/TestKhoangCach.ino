#include <Arduino.h>
#include <NimBLEDevice.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <esp_sleep.h>

// ================== Wi-Fi ==================
#define WIFI_SSID       "Tigon"
#define WIFI_PASS       "Huongloc"

// ================== RADAR (ESP32-C3) ==================
#define RADAR_PIN  GPIO_NUM_0

// ================== BLE TAGS ==================
#define COMPANY_ID_CUSTOM 0xFFFF
#define AL_TAG0 'A'
#define AL_TAG1 'L'
#define AL_TAG2 '1'
#define ACK_TAG0 'A'
#define ACK_TAG1 'C'
#define ACK_TAG2 'K'

// ================== RSSI =======================
#define RSSI_MIN_ACCEPT         (-120)
#define RSSI_ALERT_THRESHOLD    (-100)

// ================== Scan config (tối ưu nhanh) ==================
#define SCAN_MS_ACTIVE  200   // quét 200ms mỗi lần
#define SCAN_INTERVAL   40    // 40*0.625 ≈ 25ms
#define SCAN_WINDOW     40    // quét gần như liên tục

static NimBLEAdvertising* adv = nullptr;

// ================== WiFi ==================
void wifiEnsureConnected() {
  if (WiFi.status() == WL_CONNECTED) return;

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    delay(200);
  }
}

// ================== Pushcut iOS Alert ==================
bool iosAlert() {
  wifiEnsureConnected();
  if (WiFi.status() != WL_CONNECTED) return false;

  WiFiClientSecure client;
  client.setInsecure();  // bỏ kiểm tra chứng chỉ cho đơn giản

  HTTPClient https;
  if (!https.begin(client, "https://api.pushcut.io/g3_oF2B4-gHnVFZ5J9Ovf/notifications/Alert%20Child"))
    return false;

  https.addHeader("Content-Type", "application/json");

  // body rỗng cũng được, Pushcut chỉ cần gọi đúng URL
  int code = https.POST("{}");
  https.end();

  return (code >= 200 && code < 300);
}

// ================== RSSI helper ==================
static inline bool matchAck(const NimBLEAdvertisedDevice* d) {
  if (!d || !d->haveManufacturerData()) return false;
  const std::string &md = d->getManufacturerData();
  if (md.size() < 5) return false;

  uint16_t company;
  memcpy(&company, md.data(), 2);
  if (company != COMPANY_ID_CUSTOM) return false;

  if (!(md[2] == ACK_TAG0 && md[3] == ACK_TAG1 && md[4] == ACK_TAG2)) return false;
  if (d->getRSSI() < RSSI_MIN_ACCEPT) return false;

  return true;
}

const char* rssiBucket(int rssi) {
  if (rssi >= -50) return "Rất gần (~<0.5m)";
  if (rssi >= -65) return "Gần (~1–2m)";
  if (rssi >= -75) return "Trung bình (~3–4m)";
  return "Xa (>5m)";
}

// ================== AL1 Advertising ==================
void startAL1Advertising() {
  std::string mfg;
  uint16_t company = COMPANY_ID_CUSTOM;

  mfg.append((char*)&company, 2);
  mfg += "AL1";

  BLEAdvertisementData data;
  data.setFlags(0x06);
  data.setManufacturerData(mfg);

  adv->setAdvertisementData(data);

  // Quảng bá nhanh (20–30ms)
  adv->setMinInterval(32);   // ~20 ms
  adv->setMaxInterval(48);   // ~30 ms

  adv->start();
}

// ================== SLEEP (không chờ, chỉ cấu hình + sleep) ==================
void goToSleepImmediate() {
  pinMode((int)RADAR_PIN, INPUT);

  uint64_t mask = 1ULL << (int)RADAR_PIN;
  esp_deep_sleep_enable_gpio_wakeup(mask, ESP_GPIO_WAKEUP_GPIO_HIGH);

  esp_deep_sleep_start();
}

// ================== HANDLE PASS EVENT ==================
void handlePassEvent() {
  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setInterval(SCAN_INTERVAL);
  scan->setWindow(SCAN_WINDOW);
  scan->setActiveScan(true);

  bool alertSent = false;

  // RADAR HIGH = có người ở cửa
  while (digitalRead((int)RADAR_PIN) == HIGH) {
    scan->clearResults();

    // Quét 200ms, không dùng callback
    scan->start(0, false, false);
    delay(SCAN_MS_ACTIVE);
    scan->stop();

    NimBLEScanResults results = scan->getResults();
    int count = results.getCount();

    int  bestRSSI = -127;
    String bestMAC = "";
    bool hasAck = false;

    for (int i = 0; i < count; i++) {
      const NimBLEAdvertisedDevice* d = results.getDevice(i);
      if (matchAck(d)) {
        hasAck = true;
        int r = d->getRSSI();
        if (r > bestRSSI) {
          bestRSSI = r;
          bestMAC = d->getAddress().toString().c_str();
        }
      }
    }

    if (!hasAck) {
      Serial.println("[Door] Không thấy ACK → tiếp tục.");
      continue;
    }

    // Đã có ít nhất 1 ACK → dùng ACK mạnh nhất
    Serial.printf("[ACK] MAC=%s RSSI=%d → %s\n",
                  bestMAC.c_str(), bestRSSI, rssiBucket(bestRSSI));

    if (bestRSSI >= RSSI_ALERT_THRESHOLD) {
      // ACK mạnh → gửi Pushcut alert, chờ RADAR LOW rồi ngủ
      Serial.println("[Door] ALERT! Gửi Pushcut iOS...");
      iosAlert();
      alertSent = true;

      Serial.println("[Door] Đợi RADAR về LOW rồi sleep...");
      while (digitalRead((int)RADAR_PIN) == HIGH) {
        delay(20);
      }
      break;  // thoát while(RADAR HIGH)
    } else {
      Serial.println("[Door] ACK yếu → tiếp tục quét.");
      // không gửi alert, quay lại while nếu RADAR vẫn HIGH
    }
  }

  if (adv) adv->stop();

  // Ra khỏi vòng while:
  // - Trường hợp ACK mạnh: đã chờ RADAR LOW ở trên
  // - Trường hợp không có ACK mạnh: RADAR đã LOW
  Serial.println("[Door] Vào sleep.");
  goToSleepImmediate();
}

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode((int)RADAR_PIN, INPUT);

  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  Serial.print("Wake cause = ");
  Serial.println((int)cause);

  if (cause == ESP_SLEEP_WAKEUP_GPIO) {
    // Được đánh thức bởi radar → bật BLE, quảng bá AL1 và quét ACK
    NimBLEDevice::init("DoorNode-C3");
    NimBLEDevice::setPower(9);   // ~+9 dBm

    adv = NimBLEDevice::getAdvertising();
    startAL1Advertising();

    handlePassEvent();
  } else {
    // Boot lần đầu
    if (digitalRead((int)RADAR_PIN) == LOW) {
      Serial.println("[Door] Boot lần đầu, RADAR LOW → sleep ngay.");
      goToSleepImmediate();
    } else {
      Serial.println("[Door] Boot lần đầu, RADAR HIGH → xử lý như có người.");
      NimBLEDevice::init("DoorNode-C3");
      NimBLEDevice::setPower(9);

      adv = NimBLEDevice::getAdvertising();
      startAL1Advertising();

      handlePassEvent();
    }
  }
}

// ================== LOOP ==================
void loop() {}
