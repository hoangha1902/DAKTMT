#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>
#include "DHT20.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// ================= 1. CẤU HÌNH PHẦN CỨNG =================
#define I2C_SDA_PIN 11
#define I2C_SCL_PIN 12

// Pin cảm biến độ ẩm đất
#define SOIL_PIN    1 // GPIO1

// ================= 2. CẤU HÌNH MẠNG =================
#define WIFI_CHANNEL 11  

// ================= 3. CẤU HÌNH LOGIC BIÊN (EDGE CONFIG) =================
#define DIFF_SOIL 2.0   // Thay đổi >= 2% mới gửi
#define DIFF_TEMP 0.5   // Thay đổi >= 0.5 độ mới gửi
#define HEARTBEAT_MS 30000 // 30s gửi báo cáo định kỳ 1 lần

// [CALIBRATION] Cần đo thực tế để sửa 2 số này
const int DRY_VALUE = 3200; // Giá trị khi để ngoài không khí
const int WET_VALUE = 1200; // Giá trị khi nhúng vào nước

// ================= 4. KHAI BÁO BIẾN & ĐỐI TƯỢNG =================
DHT20 dht20; // Khởi tạo đối tượng DHT20

// Địa chỉ MAC của Gateway
uint8_t gatewayMac[] = {0x7C, 0xDF, 0xA1, 0x00, 0x00, 0x01};

// Địa chỉ MAC của Zone 1 (cố định để Gateway nhận diện)
uint8_t myMac[]      = {0x7C, 0xDF, 0xA1, 0x00, 0x00, 0x02};

typedef struct struct_message {
  int id;             
  float temp;         
  float hum;          
  float soil;         
} struct_message;

struct_message myData;

// Biến lưu trạng thái cũ để so sánh (Delta Check)
float lastSoil = 0;
float lastTemp = 0;
unsigned long lastSendTime = 0;

// ================= 5. HÀM ĐỌC ĐẤT (MEDIAN FILTER) =================
// Đọc 15 lần, loại bỏ nhiễu, lấy trung bình -> Cực kỳ ổn định
int readSoilMoistureStable() {
  int values[15];
  for (int i = 0; i < 15; i++) {
    values[i] = analogRead(SOIL_PIN);
    delay(10);
  }
  
  // Sắp xếp mảng (Bubble sort)
  for (int i = 0; i < 14; i++) {
    for (int j = i + 1; j < 15; j++) {
      if (values[i] > values[j]) {
        int temp = values[i];
        values[i] = values[j];
        values[j] = temp;
      }
    }
  }
  
  // Bỏ 3 mẫu đầu, 3 mẫu cuối, lấy trung bình 9 mẫu giữa
  long sum = 0;
  for (int i = 3; i < 12; i++) sum += values[i];
  int avgRaw = sum / 9;

  // Map sang phần trăm (0-100%)
  int percent = map(avgRaw, DRY_VALUE, WET_VALUE, 0, 100);
  
  // Kẹp giá trị trong khoảng 0-100
  if (percent > 100) percent = 100;
  if (percent < 0) percent = 0;
  
  return percent;
}

// ================= 6. GỬI DỮ LIỆU (ESP-NOW) =================
void sendData() {
  esp_err_t result = esp_now_send(gatewayMac, (uint8_t *) &myData, sizeof(myData));
  if (result == ESP_OK) {
    Serial.printf(">> Zone 1 Gửi OK: Soil=%.1f, Temp=%.1f\n", myData.soil, myData.temp);
    lastSoil = myData.soil;
    lastTemp = myData.temp;
    lastSendTime = millis();
  } else {
    Serial.println(">> Zone 1 Gửi Lỗi!");
  }
}

// Callback 
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {}

// ================= 7. SETUP =================
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); 
  Serial.begin(115200);
  delay(1000);

  Serial.println("=== ZONE 1 STARTING (DHT20 + Soil) ===");

  // 1. Init I2C & DHT20
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  dht20.begin(); // Khởi động DHT20
  
  // Đợi cảm biến ổn định
  delay(1000);
  if (dht20.isConnected()) {
    Serial.println("DHT20 Connected!");
  } else {
    Serial.println("DHT20 NOT found! Kiem tra day noi.");
  }

  // 2. Init Wifi & ESP-NOW
  esp_base_mac_addr_set(myMac);
  WiFi.mode(WIFI_STA);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed");
    ESP.restart();
  }
  esp_now_register_send_cb(OnDataSent);

  // 3. Đăng ký Gateway Peer
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, gatewayMac, 6);
  peerInfo.channel = WIFI_CHANNEL;  
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
  }

  // ID của Zone 1 là 1
  myData.id = 1; 
}

// ================= 8. LOOP =================
void loop() {
  // --- 1. Đọc Độ Ẩm Đất ---
  myData.soil = (float)readSoilMoistureStable();

  // --- 2. Đọc DHT20 ---
  //đọc DHT20 trước khi get giá trị
  int status = dht20.read();
  
  if (status == DHT20_OK) {
    myData.temp = dht20.getTemperature();
    myData.hum = dht20.getHumidity();
  } else {
    // Nếu đọc lỗi, giữ giá trị cũ
    Serial.print("DHT20 Error Code: ");
    Serial.println(status);
  }

  // --- 3. Logic Gửi---
  bool needSend = false;

  // Điều kiện 1: Thay đổi đột ngột (Delta)
  if (abs(myData.soil - lastSoil) >= DIFF_SOIL) needSend = true;
  if (abs(myData.temp - lastTemp) >= DIFF_TEMP) needSend = true;

  // Điều kiện 2: Định kỳ (Heartbeat)
  if (millis() - lastSendTime > HEARTBEAT_MS) needSend = true;

  if (needSend) {
    // Chống xung đột: Random 0-200ms
    delay(random(0, 200)); 
    sendData();
  }


  delay(1000);
}