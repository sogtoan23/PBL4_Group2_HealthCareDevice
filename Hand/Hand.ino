#include <Arduino.h>
#include <NimBLEDevice.h>
#include "esp_sleep.h"

// ====== Bộ lọc gói từ ESP-cửa (Manufacturer Data "AL1") ======
#define COMPANY_ID_CUSTOM 0xFFFF
#define TAG0 'A'
#define TAG1 'L'
#define TAG2 '1'
#define RSSI_MIN_ACCEPT   (-90)

// ====== ACK từ vòng tay trả về (Manufacturer Data "ACK") ======
#define ACK_TAG0 'A'
#define ACK_TAG1 'C'
#define ACK_TAG2 'K'
#define ACK_ADV_MS  300   // thời gian vòng tay phát ACK mỗi lần (ms)

// ====== Duty-cycle quét / ngủ ======
#define SCAN_MS_ACTIVE      250    // thời gian bật scan mỗi lần (ms)
#define SLEEP_MS_IDLE       3000   // nếu KHÔNG thấy cửa -> ngủ lâu (ms)
#define SLEEP_MS_ALERT       500   // nếu CÓ thấy cửa -> ngủ ngắn (ms)

// ====== Cấu hình scan BLE ======
#define SCAN_INTERVAL 45  // ~28 ms
#define SCAN_WINDOW   30  // ~18 ms

// Advertising handle cho ACK
static NimBLEAdvertising* ackAdv = nullptr;

void enterLightSleep(uint32_t sleep_ms) {
  esp_sleep_enable_timer_wakeup((uint64_t)sleep_ms * 1000ULL);
  esp_light_sleep_start();
}

static inline bool matchAlert(const NimBLEAdvertisedDevice* d) {
  if (d == nullptr) return false;
  if (!d->haveManufacturerData()) return false;

  const std::string &md = d->getManufacturerData();
  if (md.size() < 5) return false;

  uint16_t company;
  memcpy(&company, md.data(), sizeof(company));
  if (company != COMPANY_ID_CUSTOM) return false;
  if (!(md[2]==TAG0 && md[3]==TAG1 && md[4]==TAG2)) return false;
  if (d->getRSSI() < RSSI_MIN_ACCEPT) return false;
  return true;
}

// Vòng tay phát lại 1 gói ACK để cửa đo khoảng cách
void sendAckOnce() {
  if (!ackAdv) return;

  // Manufacturer Data = 0xFFFF + "ACK"
  std::string mfg;
  uint16_t company = COMPANY_ID_CUSTOM;
  mfg.append((char*)&company, 2);
  mfg += "ACK";

  BLEAdvertisementData data;
  data.setFlags(0x06);
  data.setManufacturerData(mfg);

  ackAdv->setAdvertisementData(data);
  ackAdv->start();
  Serial.println("[WB] Send ACK adv");

  delay(ACK_ADV_MS);

  ackAdv->stop();
  Serial.println("[WB] Stop ACK adv");
}

void setup() {
  Serial.begin(115200);
  delay(200);

  NimBLEDevice::init("Wristband");
  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setInterval(SCAN_INTERVAL);
  scan->setWindow(SCAN_WINDOW);
  scan->setActiveScan(true);
  scan->setDuplicateFilter(true);

  // chuẩn bị advertising cho ACK
  ackAdv = NimBLEDevice::getAdvertising();

  Serial.println("[WB] Ready: scan AL1, then send ACK when detected (with sleep).");
}

void loop() {
  NimBLEScan* scan = NimBLEDevice::getScan();

  // 1) Bật scan
  scan->start(0 /*seconds*/, false /*isContinue*/, false /*restart*/);
  delay(SCAN_MS_ACTIVE);
  scan->stop();

  bool detected = false;

  // 2) Duyệt kết quả
  NimBLEScanResults results = scan->getResults();
  int count = results.getCount();
  for (int i = 0; i < count; ++i) {
    const NimBLEAdvertisedDevice* d = results.getDevice(i);
    if (matchAlert(d)) {
      detected = true;
      Serial.printf("[DETECT] AL1 from door | MAC=%s  RSSI=%d dBm\n",
                    d->getAddress().toString().c_str(),
                    d->getRSSI());
    }
  }
  scan->clearResults(); // giải phóng bộ nhớ

  // 3) Nếu thấy cửa -> phát ACK
  if (detected) {
    sendAckOnce();
  }

  // 4) Ngủ: nếu có -> ngủ ngắn, không có -> ngủ lâu
  uint32_t sleepMs = detected ? SLEEP_MS_ALERT : SLEEP_MS_IDLE;
  Serial.printf("[WB] Sleep %lu ms (detected=%d)\n", (unsigned long)sleepMs, detected);
  enterLightSleep(sleepMs);
}
