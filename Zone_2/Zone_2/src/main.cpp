#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// ================= 1. CẤU HÌNH PHẦN CỨNG =================
#define I2C_SDA_PIN 11
#define I2C_SCL_PIN 12

// Giữ nguyên PIN 1 theo yêu cầu của bạn
#define SOIL_PIN    1 // Cảm biến độ ẩm đất nối vào ADC1_1 (PIN 1 / GPIO1)   

// ================= 2. CẤU HÌNH MẠNG =================
#define WIFI_CHANNEL 6  // Bắt buộc trùng Gateway

// ================= 3. CẤU HÌNH LOGIC BIÊN (EDGE CONFIG) =================
// Ngưỡng thay đổi để kích hoạt gửi (Delta)
#define DIFF_SOIL 2.0   // Chỉ gửi nếu độ ẩm đất lệch >= 2%
#define DIFF_TEMP 0.5   // Chỉ gửi nếu nhiệt độ lệch >= 0.5 độ
#define HEARTBEAT_MS 30000 // 30 giây gửi 1 lần (Heartbeat)

// Mốc hiệu chỉnh (Bạn tự sửa lại số này theo thực tế của Zone 2 nhé)
const int DRY_VALUE = 3200; 
const int WET_VALUE = 1200; 

// ================= 4. ĐỊA CHỈ MAC & DATA =================
uint8_t gatewayMac[] = {0x7C, 0xDF, 0xA1, 0x00, 0x00, 0x01};

// [THAY ĐỔI 1] MAC của Zone 2 phải khác Zone 1 (Đuôi :03)
uint8_t myMac[]      = {0x7C, 0xDF, 0xA1, 0x00, 0x00, 0x03}; 

typedef struct {
  int id;       
  float temp;   
  float hum;
  float soil;   
} struct_message;

struct_message myData;
Adafruit_AHTX0 aht; 

// Biến lưu trạng thái cũ để so sánh
float lastSentSoil = -100.0;
float lastSentTemp = -100.0;
unsigned long lastSentTime = 0;

// ================= 5. HÀM XỬ LÝ (ALGORITHMS) =================

// --- THUẬT TOÁN 1: LỌC TRUNG VỊ (MEDIAN FILTER) ---
// Giúp loại bỏ nhiễu gai, làm mượt dữ liệu cảm biến đất
int readSoilMoistureStable() {
  const int SAMPLES = 15;
  int rawValues[SAMPLES];

  // A. Lấy 15 mẫu liên tiếp
  for (int i = 0; i < SAMPLES; i++) {
    rawValues[i] = analogRead(SOIL_PIN);
    delay(10); 
  }

  // B. Sắp xếp từ bé đến lớn (Bubble Sort)
  for (int i = 0; i < SAMPLES - 1; i++) {
    for (int j = i + 1; j < SAMPLES; j++) {
      if (rawValues[i] > rawValues[j]) {
        int temp = rawValues[i];
        rawValues[i] = rawValues[j];
        rawValues[j] = temp;
      }
    }
  }

  // C. Bỏ 3 mẫu nhỏ nhất và 3 mẫu lớn nhất -> Lấy trung bình 9 mẫu giữa
  long sum = 0;
  for (int i = 3; i < 12; i++) {
    sum += rawValues[i];
  }
  int avgRaw = sum / 9;

  // D. Map sang %
  int percent = map(avgRaw, DRY_VALUE, WET_VALUE, 0, 100);
  if (percent > 100) percent = 100;
  if (percent < 0) percent = 0;

  return percent;
}

// --- HÀM GỬI DỮ LIỆU ---
void sendData(bool isHeartbeat) {
  // [THAY ĐỔI 2] ID của Zone 2 là 2
  myData.id = 2; 
  
  esp_err_t result = esp_now_send(gatewayMac, (uint8_t *) &myData, sizeof(myData));
  
  if (result == ESP_OK) {
    if (isHeartbeat) Serial.print("[Heartbeat] ");
    else Serial.print("[Change] ");
    
    Serial.printf("Gửi OK (Zone 2): Soil=%d%%, Temp=%.2f\n", (int)myData.soil, myData.temp);

    // Cập nhật "ký ức"
    lastSentSoil = myData.soil;
    lastSentTemp = myData.temp;
    lastSentTime = millis();
  } else {
    Serial.println("Gửi Lỗi");
  }
}

// Callback
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Không in gì ở đây để đỡ rối monitor
}

// ================= 6. SETUP =================
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); 
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n=== ZONE 2: EDGE COMPUTING ENABLED ===");

  // Init Hardware
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  if (!aht.begin()) Serial.println("Lỗi DHT20");
  pinMode(SOIL_PIN, INPUT);

  // Set MAC & WiFi
  esp_base_mac_addr_set(myMac);
  WiFi.mode(WIFI_STA);
  esp_wifi_set_max_tx_power(50);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) return;
  esp_now_register_send_cb(OnDataSent);

  // Add Gateway Peer
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, gatewayMac, 6);
  peerInfo.channel = WIFI_CHANNEL;  
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
}

// ================= 7. LOOP (LOGIC THÔNG MINH) =================
void loop() {
  // 1. Đọc & Lọc nhiễu Đất (Median Filter)
  myData.soil = (float)readSoilMoistureStable();

  // 2. Đọc & Kiểm tra DHT20 (Validation)
  sensors_event_t humidity, temp;
  if (aht.begin()) {
    aht.getEvent(&humidity, &temp);
    // Chỉ nhận nếu nhiệt độ hợp lý (-10 đến 80 độ) để lọc giá trị rác
    if (!isnan(temp.temperature) && temp.temperature > -10 && temp.temperature < 80) {
      myData.temp = temp.temperature;
      myData.hum = humidity.relative_humidity;
    }
  }

  // 3. CHIẾN LƯỢC QUYẾT ĐỊNH GỬI (Decision Making)
  
  // A. Kiểm tra sự thay đổi (Delta Check)
  bool soilChanged = abs(myData.soil - lastSentSoil) >= DIFF_SOIL;
  bool tempChanged = abs(myData.temp - lastSentTemp) >= DIFF_TEMP;

  // B. Kiểm tra thời gian chờ (Heartbeat Check)
  bool timeOut = (millis() - lastSentTime) >= HEARTBEAT_MS;

  // C. Quyết định
  if (soilChanged || tempChanged || timeOut) {
    // [THAY ĐỔI 3] Anti-collision: Thêm độ trễ ngẫu nhiên để tránh đụng độ với Zone 1
    delay(random(50, 200)); 
    sendData(timeOut); 
  } 

  delay(1000); // Đọc 1s/lần
}