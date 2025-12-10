#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// ================= MODULE 1: CẤU HÌNH PHẦN CỨNG =================
// Output: Relay kích hoạt máy bơm (Kích High hoặc Low tùy module)
#define RELAY_1_PIN 4   // Bơm Zone 1
#define RELAY_2_PIN 5   // Bơm Zone 2

// Input: Cảm biến lưu lượng (YF-S201: 1 Lít = 450 xung)
#define FLOW_1_PIN  6   // Đo nước ra Zone 1
#define FLOW_2_PIN  7   // Đo nước ra Zone 2

// Input: Giả lập phao nước (Nối đất = Cạn, Thả nổi = Đầy)
// Dùng GPIO 10 với chế độ INPUT_PULLUP
#define WATER_LEVEL_PIN 10 

// ================= MODULE 2: CẤU HÌNH MẠNG =================
#define WIFI_CHANNEL 6  // Bắt buộc trùng Gateway
#define MY_ID        0  // ID 0 dành riêng cho Trạm Trung Tâm

// MAC Gateway (...:01)
uint8_t gatewayMac[] = {0x7C, 0xDF, 0xA1, 0x00, 0x00, 0x01};

// MAC của Node Bể Chứa (Đuôi ...:05 cho khác biệt)
uint8_t myMac[]      = {0x7C, 0xDF, 0xA1, 0x00, 0x00, 0x05}; 

// ================= MODULE 3: BIẾN TOÀN CỤC & STRUCT =================

// Struct GỬI đi (Phải khớp Gateway để không bị lỗi)
// Mẹo: Tận dụng các biến có sẵn để chứa dữ liệu nước
// temp -> Chứa Tốc độ chảy (L/min)
// hum  -> Chứa Tổng lượng nước (Lít)
// soil -> Chứa Trạng thái Relay (0=Tắt, 1=Bật, 2=Lỗi)
typedef struct {
  int id;       
  float flow_rate;   // (Mapping vào temp bên Gateway)
  float total_vol;   // (Mapping vào hum bên Gateway)
  float status;      // (Mapping vào soil bên Gateway)
} struct_message;

struct_message myData;

// Struct NHẬN lệnh (Command từ Gateway)
typedef struct {
  int cmd_id;    // 1: Điều khiển Bơm
  int pump_id;   // 1 hoặc 2
  int action;    // 1: Bật, 0: Tắt
} struct_command;

struct_command incomingCmd;

// Biến cho Cảm biến lưu lượng (VOLATILE là bắt buộc cho ngắt)
volatile long pulseCount1 = 0;
volatile long pulseCount2 = 0;
float flowRate1 = 0.0, flowRate2 = 0.0;
float totalVol1 = 0.0, totalVol2 = 0.0;

unsigned long oldTime = 0;
const float CALIBRATION_FACTOR = 7.5; // Tùy chỉnh theo loại cảm biến (7.5 cho YF-S201)

// Biến bảo vệ (Safety)
bool pump1State = false;
bool pump2State = false;
bool pump1Error = false; // Cờ báo lỗi chạy khô
unsigned long pump1StartTime = 0; // Thời điểm bắt đầu bật

// ================= MODULE 4: XỬ LÝ NGẮT (INTERRUPT) =================
// Hàm này chạy cực nhanh mỗi khi cánh quạt quay 1 vòng
void IRAM_ATTR pulseCounter1() {
  pulseCount1++;
}
void IRAM_ATTR pulseCounter2() {
  pulseCount2++;
}

// ================= MODULE 5: GIAO TIẾP ESP-NOW =================

// Hàm Gửi Báo Cáo
void sendReport() {
  myData.id = MY_ID;
  
  // Gói dữ liệu Zone 1 (Mặc định báo cáo Zone 1 trước)
  // Trong thực tế có thể gửi mảng hoặc gửi lần lượt
  myData.flow_rate = flowRate1; 
  myData.total_vol = totalVol1;
  
  if (pump1Error) myData.status = 2.0; // 2 = Lỗi
  else myData.status = pump1State ? 1.0 : 0.0;

  esp_err_t result = esp_now_send(gatewayMac, (uint8_t *) &myData, sizeof(myData));
  
  if (result == ESP_OK) Serial.println(">> Gửi báo cáo OK");
  else Serial.println(">> Gửi Lỗi");
}

// Hàm Nhận Lệnh (Callback)
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  // Vì Gateway hiện tại chưa gửi lệnh xuống (chưa code phần RPC),
  // nên hàm này để chờ (Placeholder) cho tính năng mở rộng sau này.
  // Khi bạn code Gateway gửi lệnh, ta sẽ parse struct_command ở đây.
  Serial.print("Nhận lệnh: ");
  Serial.println(len);
}

// ================= MODULE 6: LOGIC ĐIỀU KHIỂN & BẢO VỆ =================

void calculateFlowAndProtect() {
  // Chỉ tính toán mỗi 1 giây (1000ms)
  if ((millis() - oldTime) > 1000) {
    
    // 1. Ngắt đếm tạm thời để đọc dữ liệu
    detachInterrupt(digitalPinToInterrupt(FLOW_1_PIN));
    
    // 2. Tính toán: (Số xung / 7.5) = Lít/phút
    flowRate1 = ((1000.0 / (millis() - oldTime)) * pulseCount1) / CALIBRATION_FACTOR;
    
    // 3. Cộng dồn tổng tích lũy (Lít) -> Chia 60 vì flowRate là L/phút
    totalVol1 += (flowRate1 / 60.0);
    
    // 4. LOGIC BẢO VỆ CHẠY KHÔ (Dry Run Protection)
    if (pump1State && !pump1Error) {
      // Nếu bơm đang bật
      unsigned long runTime = millis() - pump1StartTime;
      
      // Sau 10 giây khởi động mà nước vẫn không chảy (Flow < 1.0)
      if (runTime > 10000 && flowRate1 < 1.0) {
        Serial.println("!!! CẢNH BÁO: BƠM CHẠY KHÔ -> NGẮT KHẨN CẤP !!!");
        digitalWrite(RELAY_1_PIN, LOW); // Ngắt relay cứng
        pump1State = false;
        pump1Error = true; // Set cờ lỗi
      }
    }

    // 5. Reset xung và nạp lại ngắt
    oldTime = millis();
    pulseCount1 = 0;
    attachInterrupt(digitalPinToInterrupt(FLOW_1_PIN), pulseCounter1, FALLING);
    
    // In ra Monitor
    Serial.printf("[Bể Chứa] Flow: %.2f L/min | Total: %.2f L | Pump: %d\n", 
                  flowRate1, totalVol1, pump1State);
    
    // Gửi báo cáo về Gateway
    sendReport();
  }
}

// Hàm bật tắt bơm (Dùng để test)
void togglePump1() {
  if (pump1Error) {
    Serial.println("Đang lỗi, cần Reset hệ thống mới được bơm lại!");
    return;
  }
  
  pump1State = !pump1State;
  digitalWrite(RELAY_1_PIN, pump1State ? HIGH : LOW);
  
  if (pump1State) {
    pump1StartTime = millis(); // Ghi lại giờ bắt đầu để tính timeout
    Serial.println("-> BẬT BƠM 1");
  } else {
    Serial.println("-> TẮT BƠM 1");
  }
}

// ================= SETUP =================
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); 
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== NODE BỂ CHỨA (CENTRAL STATION) ===");

  // 1. Cấu hình PIN
  pinMode(RELAY_1_PIN, OUTPUT);
  pinMode(RELAY_2_PIN, OUTPUT);
  digitalWrite(RELAY_1_PIN, LOW); // Mặc định tắt
  digitalWrite(RELAY_2_PIN, LOW);

  pinMode(FLOW_1_PIN, INPUT_PULLUP);
  pinMode(WATER_LEVEL_PIN, INPUT_PULLUP); // Nối đất = Cạn (LOW)

  // 2. Cấu hình Ngắt
  attachInterrupt(digitalPinToInterrupt(FLOW_1_PIN), pulseCounter1, FALLING);
  
  // 3. ESP-NOW Init
  esp_base_mac_addr_set(myMac);
  WiFi.mode(WIFI_STA);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) return;
  
  // Đăng ký nhận lệnh
  esp_now_register_recv_cb(OnDataRecv);

  // Add Gateway Peer (Để gửi báo cáo)
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, gatewayMac, 6);
  peerInfo.channel = WIFI_CHANNEL;  
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
}

// ================= LOOP =================
void loop() {
  // 1. Tính toán & Bảo vệ (Chạy mỗi 1s)
  calculateFlowAndProtect();

  // 2. Giả lập Lệnh điều khiển (Vì Gateway chưa gửi lệnh)
  // Cơ chế: Chập chân GPIO 10 xuống đất để Reset lỗi hoặc Test bơm
  if (digitalRead(WATER_LEVEL_PIN) == LOW) {
    delay(100); // Chống rung
    // Nếu đang lỗi thì Reset lỗi
    if (pump1Error) {
      pump1Error = false;
      Serial.println("-> Đã Reset Lỗi!");
    } else {
      // Nếu không lỗi thì bật/tắt bơm để test
      togglePump1();
      delay(2000); // Delay để không bật tắt liên tục
    }
  }
}