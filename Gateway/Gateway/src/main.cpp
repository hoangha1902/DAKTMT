#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <time.h>
#include <math.h> 
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// ================= 1. CẤU HÌNH HỆ THỐNG =================
const char* WIFI_SSID       = "SJQKA";
const char* WIFI_PASSWORD   = "1236547890";

const char* MQTT_SERVER     = "app.coreiot.io";
const int   MQTT_PORT       = 1883;
const char* MQTT_TOKEN      = "UBXoN28TXw7FLwqP0ocy";

#define WIFI_CHANNEL            11 

// Cấu hình Logic Offline
#define SOIL_DRY_THRESHOLD      30.0f
#define SOIL_WET_THRESHOLD      60.0f
#define OFFLINE_TIMEOUT         60000UL
#define RECONNECT_INTERVAL      30000UL

// ================= 2. MAC ADDRESS =================
// Địa chỉ MAC của Gateway 
uint8_t myBaseMac[]      = {0x7C, 0xDF, 0xA1, 0x00, 0x00, 0x01};
// Địa chỉ MAC của Node Bể chứa nước 
uint8_t centralNodeMac[] = {0x7C, 0xDF, 0xA1, 0x00, 0x00, 0x05};

// ================= 3. STRUCT & VARS =================
typedef struct {
  int   id;
  float temp;
  float hum;
  float soil;
} struct_message;

struct_message incomingData;

typedef struct {
  int cmd_id;
  int pump_id;
  int action;
} struct_command;

struct_command cmd;

WiFiClient espClient;
PubSubClient client(espClient);

// Biến trạng thái mạng
unsigned long lastWiFiCheckTime    = 0;
unsigned long lastReconnectAttempt = 0;
bool isOfflineMode = false;

// Cache Logic Offline
float lastSoilZone1 = 100.0f;
float lastSoilZone2 = 100.0f;
bool pump1Status = false;
bool pump2Status = false;

// Cờ lỗi tổng
bool centralError = false;
bool centralSumError = false;
bool pump1Error = false;
bool pump2Error = false;

// Cache dữ liệu Central (để debug)
float pump1Flow = 0.0f, pump2Flow = 0.0f;
float pump1Total = 0.0f, pump2Total = 0.0f;
// ================= DAILY VOLUME (computed at Gateway) =================
// Baseline totals at start of local day (yyyyMMdd)
static int    g_dayKey = -1;
static float  g_pump1Day0 = 0.0f;
static float  g_pump2Day0 = 0.0f;
static float  g_pump1Daily = 0.0f;
static float  g_pump2Daily = 0.0f;

// Return local date key (yyyyMMdd). If time not synced, returns -1.
static int getLocalDayKey() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 10)) {
    return -1;
  }
  int y = timeinfo.tm_year + 1900;
  int m = timeinfo.tm_mon + 1;
  int d = timeinfo.tm_mday;
  return y * 10000 + m * 100 + d;
}

static void updateDailyVolumes() {
  int dk = getLocalDayKey();
  // If time not ready, we keep daily volumes as "since boot baseline"
  if (dk < 0) {
    if (g_dayKey < 0) {
      g_dayKey = 0;          // special "unknown day"
      g_pump1Day0 = pump1Total;
      g_pump2Day0 = pump2Total;
    }
  } else if (dk != g_dayKey) {
    g_dayKey = dk;
    g_pump1Day0 = pump1Total;
    g_pump2Day0 = pump2Total;
  }

  g_pump1Daily = pump1Total - g_pump1Day0;
  g_pump2Daily = pump2Total - g_pump2Day0;
  if (g_pump1Daily < 0) g_pump1Daily = 0;
  if (g_pump2Daily < 0) g_pump2Daily = 0;
}

// Cấu hình Health Check
#define NODE_TIMEOUT_MS         60000UL // 2 phút không gửi -> Báo OFFLINE
#define HEALTH_CHECK_INTERVAL   10000UL  // Kiểm tra mỗi 10s

// Device ảo khí hậu chung
#define ENV_DEVICE_NAME         "Env_Area"
#define ENV_STALE_MS            120000UL
#define ENV_PUB_INTERVAL_MS     2000UL

// Gateway self-status (for LWT + dashboards)
#define GW_ATTR_TOPIC            "v1/devices/me/attributes"
#define GW_TLM_TOPIC             "v1/devices/me/telemetry"
#define GW_STATUS_ONLINE         "{\"connection_status\":\"ONLINE\"}"
#define GW_STATUS_OFFLINE        "{\"connection_status\":\"OFFLINE\"}"
#define GW_HEARTBEAT_INTERVAL_MS 10000UL  // 10s: used for 3A (last-update based offline)

// ================= 1.1. CENTRAL EXTENDED IDs =================
#define ID_CENTRAL_SUM          0
#define ID_PUMP1                10
#define ID_PUMP2                11
#define ID_TANK                 12

#define PUMP_BIT_ON             0x01
#define PUMP_BIT_DRYRUN         0x02
#define PUMP_BIT_RELAY_STUCK    0x04
#define TANK_FLAG_LOW_PCT       0x01
#define TANK_FLAG_LOW_SWITCH    0x02


int   pump1StatusBits = 0, pump2StatusBits = 0;
float tankLevelPct = NAN;
float tankLevelL   = NAN;
int   tankFlagsBits = 0;

// --- BIẾN HEALTH CHECK ---
unsigned long lastSeenZone1 = 0;
unsigned long lastSeenZone2 = 0;
unsigned long lastSeenCentral = 0;

bool isZone1Online = true;
bool isZone2Online = true;
bool isCentralOnline = true;

// Env Area Cache
float lastTempZ1 = NAN, lastHumZ1 = NAN;
float lastTempZ2 = NAN, lastHumZ2 = NAN;
unsigned long lastEnvTsZ1 = 0, lastEnvTsZ2 = 0;
unsigned long lastEnvPublish = 0;

unsigned long lastGwHeartbeat = 0;
// ================= 6. CÁC HÀM PHỤ TRỢ (HELPER) =================

// Tính trung bình môi trường
bool getEnvAvg(float &tAvg, float &hAvg) {
  unsigned long now = millis();
  bool z1ok = (lastEnvTsZ1 != 0) && (now - lastEnvTsZ1 <= ENV_STALE_MS) && !isnan(lastTempZ1);
  bool z2ok = (lastEnvTsZ2 != 0) && (now - lastEnvTsZ2 <= ENV_STALE_MS) && !isnan(lastTempZ2);

  if (z1ok && z2ok) {
    tAvg = (lastTempZ1 + lastTempZ2) / 2.0f;
    hAvg = (lastHumZ1 + lastHumZ2) / 2.0f;
    return true;
  }
  if (z1ok) { tAvg = lastTempZ1; hAvg = lastHumZ1; return true; }
  if (z2ok) { tAvg = lastTempZ2; hAvg = lastHumZ2; return true; }
  return false;
}

void publishEnvAvgIfReady() {
  if (isOfflineMode || !client.connected()) return;
  unsigned long now = millis();
  if (now - lastEnvPublish < ENV_PUB_INTERVAL_MS) return;

  float tAvg, hAvg;
  if (!getEnvAvg(tAvg, hAvg)) return;

  // [FIX] Tăng buffer lên 2048 cho an toàn
  DynamicJsonDocument doc(2048);
  doc[ENV_DEVICE_NAME][0]["temperature"] = tAvg;
  doc[ENV_DEVICE_NAME][0]["humidity"]    = hAvg;
  String output;
  serializeJson(doc, output);
  client.publish("v1/gateway/telemetry", output.c_str());
  lastEnvPublish = now;
}

// Hàm báo cáo trạng thái Online/Offline lên Cloud

// Publish gateway heartbeat (telemetry) so dashboards can infer ONLINE/OFFLINE from last update time
void publishGatewayHeartbeat() {
  if (isOfflineMode || !client.connected()) return;

  unsigned long now = millis();
  if (now - lastGwHeartbeat < GW_HEARTBEAT_INTERVAL_MS) return;
  lastGwHeartbeat = now;

  DynamicJsonDocument doc(256);
  static uint32_t hbCounter = 0;
  doc["hb"] = ++hbCounter;
  doc["rssi"] = WiFi.isConnected() ? WiFi.RSSI() : 0;
  doc["uptime_s"] = (unsigned long)(millis() / 1000);

  String out;
  serializeJson(doc, out);
  client.publish(GW_TLM_TOPIC, out.c_str(), false);
}

void reportDeviceStatus(String deviceName, bool isOnline) {
  if (isOfflineMode || !client.connected()) return;

  DynamicJsonDocument doc(2048);
  doc[deviceName][0]["active"] = isOnline ? 1 : 0;
  doc[deviceName][0]["connection_status"] = isOnline ? "ONLINE" : "OFFLINE";

  String output;
  serializeJson(doc, output);
  client.publish("v1/gateway/telemetry", output.c_str());
  
  Serial.printf(">> [HEALTH] Báo cáo %s: %s\n", deviceName.c_str(), isOnline ? "ONLINE" : "OFFLINE");
}

// ================= 7. GỬI LỆNH ESPNOW (ĐÃ SỬA INTERLOCK) =================
// Hàm trả về bool để biết gửi thành công hay thất bại
bool sendPumpCommand(int pumpId, bool turnOn) {
  // 1. Kiểm tra Interlock (Chặn lệnh nếu có lỗi nghiêm trọng)
  if (centralError && turnOn) {
    Serial.println("!!! BLOCKED: Central Error đang kích hoạt. Không thể BẬT bơm! !!!");
    return false; // Trả về false để báo lệnh thất bại
  }

  cmd.cmd_id  = 1;
  cmd.pump_id = pumpId;
  cmd.action  = turnOn ? 1 : 0;
  
  esp_err_t result = esp_now_send(centralNodeMac, (uint8_t *)&cmd, sizeof(cmd));
  
  if (result == ESP_OK) {
    Serial.printf("[CMD] Gửi thành công Pump%d -> %s\n", pumpId, turnOn ? "ON" : "OFF");
    return true;
  } else {
    Serial.println("[CMD] Gửi ESPNOW thất bại!");
    return false;
  }
}

// ================= 8. CALLBACK NHẬN DỮ LIỆU (ON DATA RECV) =================
void OnDataRecv(const uint8_t * mac, const uint8_t *incoming, int len) {
  // Safety: ignore unexpected ESP-NOW payload sizes
  if (len < sizeof(incomingData)) {
    Serial.printf("[ESPNOW] Packet too small: %d bytes (need >= %u), len, (unsigned)sizeof(incomingData)");
    return;
  }
  memcpy(&incomingData, incoming, sizeof(incomingData));
unsigned long now = millis();

  // --- CẬP NHẬT HEARTBEAT ---
  if (incomingData.id == 1) {
    lastSeenZone1 = now;
    if (!isZone1Online) { isZone1Online = true; reportDeviceStatus("Zone_1", true); }
    
    lastSoilZone1 = incomingData.soil;
    lastTempZ1 = incomingData.temp; lastHumZ1 = incomingData.hum; lastEnvTsZ1 = now;

  } else if (incomingData.id == 2) {
    lastSeenZone2 = now;
    if (!isZone2Online) { isZone2Online = true; reportDeviceStatus("Zone_2", true); }

    lastSoilZone2 = incomingData.soil;
    lastTempZ2 = incomingData.temp; lastHumZ2 = incomingData.hum; lastEnvTsZ2 = now;

  } else if (incomingData.id == ID_CENTRAL_SUM || incomingData.id >= 10) {
    // Bất kỳ gói tin nào từ Central đều tính là sống
    lastSeenCentral = now;
    if (!isCentralOnline) { isCentralOnline = true; reportDeviceStatus("Central_Station", true); }
    
    // Cập nhật biến cache Central 
    if (incomingData.id == ID_CENTRAL_SUM) {
       // Làm tròn  tránh lỗi số học float 
       centralSumError = ((int)round(incomingData.soil) == 2);
    } else if (incomingData.id == ID_PUMP1) {
       pump1Flow = incomingData.temp; pump1Total = incomingData.hum;
       pump1StatusBits = (int)round(incomingData.soil); 
       pump1Error = ((pump1StatusBits & PUMP_BIT_DRYRUN) != 0);
    } else if (incomingData.id == ID_PUMP2) {
       pump2Flow = incomingData.temp; pump2Total = incomingData.hum;
       pump2StatusBits = (int)round(incomingData.soil); 
       pump2Error = ((pump2StatusBits & PUMP_BIT_DRYRUN) != 0);
    } else if (incomingData.id == ID_TANK) {
       tankLevelPct = incomingData.temp; tankLevelL = incomingData.hum;
       tankFlagsBits = (int)round(incomingData.soil); 
    }
  }

  
  centralError = (centralSumError || pump1Error || pump2Error);

  // Update daily volume baselines & values (uses pump1Total/pump2Total caches)
  updateDailyVolumes();

  // --- GỬI JSON TELEMETRY LÊN CLOUD ---
  if (!isOfflineMode && client.connected()) {
    DynamicJsonDocument doc(2048); 
    String deviceName;

    // Phân loại Central Station
    if (incomingData.id == ID_CENTRAL_SUM || incomingData.id == ID_PUMP1 || 
        incomingData.id == ID_PUMP2 || incomingData.id == ID_TANK) {
      
      deviceName = "Central_Station";
      JsonArray arr = doc.createNestedArray(deviceName);
      JsonObject obj = arr.createNestedObject();

      if (incomingData.id == ID_CENTRAL_SUM) {
        obj["flow_rate"]    = incomingData.temp;
        obj["total_volume"] = incomingData.hum;
        obj["daily_volume"] = (g_pump1Daily + g_pump2Daily);
        obj["day_key"] = g_dayKey;
        obj["pump_status"]  = (int)round(incomingData.soil);
      }
      else if (incomingData.id == ID_PUMP1) {
        int bits = (int)round(incomingData.soil); 
        obj["pump1_flow_rate"]    = incomingData.temp;
        obj["pump1_total_volume"] = incomingData.hum;
        obj["pump1_daily_volume"] = g_pump1Daily;
        obj["day_key"] = g_dayKey;
        obj["pump1_status_bits"]  = bits;
        obj["pump1_on"]           = ((bits & PUMP_BIT_ON) != 0);
        obj["pump1_error"]        = ((bits & PUMP_BIT_DRYRUN) != 0);
        obj["pump1_relay_stuck"]  = ((bits & PUMP_BIT_RELAY_STUCK) != 0);
      }
      else if (incomingData.id == ID_PUMP2) {
        int bits = (int)round(incomingData.soil);
        obj["pump2_flow_rate"]    = incomingData.temp;
        obj["pump2_total_volume"] = incomingData.hum;
        obj["pump2_daily_volume"] = g_pump2Daily;
        obj["day_key"] = g_dayKey;
        obj["pump2_status_bits"]  = bits;
        obj["pump2_on"]           = ((bits & PUMP_BIT_ON) != 0);
        obj["pump2_error"]        = ((bits & PUMP_BIT_DRYRUN) != 0);
        obj["pump2_relay_stuck"]  = ((bits & PUMP_BIT_RELAY_STUCK) != 0);
      }
      else if (incomingData.id == ID_TANK) {
        int bits = (int)round(incomingData.soil); // [FIX] round()
        obj["tank_level_pct"] = incomingData.temp;
        obj["tank_level_l"]   = incomingData.hum;
        obj["tank_flags_bits"]= bits;
        obj["tank_low_pct"]   = ((bits & TANK_FLAG_LOW_PCT) != 0);
        obj["tank_low_switch"]= ((bits & TANK_FLAG_LOW_SWITCH) != 0);
      }
    } 
    // Phân loại Zone
    else {
      deviceName = "Zone_" + String(incomingData.id);
      JsonArray arr = doc.createNestedArray(deviceName);
      JsonObject obj = arr.createNestedObject();
      obj["temperature"]   = incomingData.temp;
      obj["humidity"]      = incomingData.hum;
      obj["soil_moisture"] = incomingData.soil;
    }
    
    String output;
    serializeJson(doc, output);
    client.publish("v1/gateway/telemetry", output.c_str());

    // Nếu là Zone 1/2 thì cập nhật cả Env_Area
    if (incomingData.id == 1 || incomingData.id == 2) {
      publishEnvAvgIfReady();
    }
  }
}

// ================= 9. MQTT RPC CALLBACK =================
void callback(char* topic, byte* payload, unsigned int length) {
  DynamicJsonDocument doc(2048);
  DeserializationError err = deserializeJson(doc, payload, length);
  
  if (err) {
    Serial.println("Lỗi Parse RPC JSON");
    return;
  }

  String method;
  bool params = false;
  bool valid = false;

  if (doc.containsKey("method")) {
    method = doc["method"].as<String>();
    params = doc["params"].as<bool>();
    valid = true;
  } else if (doc.containsKey("device") && doc.containsKey("data")) {
    JsonObject data = doc["data"].as<JsonObject>();
    if (data.containsKey("method")) {
      method = data["method"].as<String>();
      params = data["params"].as<bool>();
      valid = true;
    }
  }

  if (!valid) return;

  // Xử lý lệnh khẩn cấp trước  lệnh thường
  // ============================================
  if (method == "forceOffAll" || method == "emergencyStop") {
    Serial.println("!!! [CLOUD EMERGENCY] Force OFF all pumps !!!");
    
    // Tắt cả 2 bơm ngay lập tức
    if (sendPumpCommand(1, false)) pump1Status = false;
    if (sendPumpCommand(2, false)) pump2Status = false;
    
    return;
  }
  // ============================================
  int pumpId = 0;
  if (method == "setPump1") pumpId = 1;
  else if (method == "setPump2") pumpId = 2;

  if (pumpId > 0) {
    // Chỉ cập nhật trạng thái UI giả định NẾU gửi lệnh thành công
    bool success = sendPumpCommand(pumpId, params);
    
    if (success) {
      if (pumpId == 1) pump1Status = params;
      if (pumpId == 2) pump2Status = params;
      Serial.println("=> [UI UPDATE] Đã cập nhật trạng thái giả định.");
    } else {
      Serial.println("=> [UI ERROR] Lệnh bị chặn hoặc lỗi gửi. KHÔNG cập nhật trạng thái.");
    }
  }
}

// ================= 10. OFFLINE LOGIC =================

void runOfflineLogic() {
  static unsigned long lastOfflineLoop = 0;
  if (millis() - lastOfflineLoop < 5000) return; // Chạy mỗi 5 giây
  lastOfflineLoop = millis();

  Serial.println(">>> [OFFLINE MODE] <<<");

  // 1. LỚP AN TOÀN: Kiểm tra lỗi phần cứng
  if (centralError) {
    Serial.println("!!! Central Error (Cạn bể/Lỗi bơm) -> STOP ALL !!!");
    // Nếu đang bật thì tắt ngay
    if (pump1Status) { sendPumpCommand(1, false); pump1Status = false; }
    if (pump2Status) { sendPumpCommand(2, false); pump2Status = false; }
    return;
  }

  // 2. LỚP MÔI TRƯỜNG: Kiểm tra Thời tiết trung bình
  float tAvg, hAvg;
  if (getEnvAvg(tAvg, hAvg)) {
    Serial.printf("[OFFLINE] Avg Temp: %.1f, Hum: %.1f\n", tAvg, hAvg);
    
    // Ngưỡng: Ẩm > 90% hoặc Nhiệt < 18 độ -> CẤM TƯỚI
    if (hAvg > 90.0 || tAvg < 18.0) {
       Serial.println("!!! THỜI TIẾT KHÔNG PHÙ HỢP -> HỦY TƯỚI !!!");
       if (pump1Status) { sendPumpCommand(1, false); pump1Status = false; }
       if (pump2Status) { sendPumpCommand(2, false); pump2Status = false; }
       return; 
    }
  }

  // 3. LỚP CỤC BỘ: Kiểm tra Đất từng Zone 
  
  // --- Zone 1 ---
  if (lastSoilZone1 < SOIL_DRY_THRESHOLD && !pump1Status) {
    Serial.printf("Zone 1 Khô (%.1f%%) -> BẬT Bơm 1\n", lastSoilZone1);
    if (sendPumpCommand(1, true)) pump1Status = true;
  } 
  else if (lastSoilZone1 > SOIL_WET_THRESHOLD && pump1Status) {
    Serial.printf("Zone 1 Ẩm (%.1f%%) -> TẮT Bơm 1\n", lastSoilZone1);
    if (sendPumpCommand(1, false)) pump1Status = false;
  }

  // --- Zone 2 ---
  if (lastSoilZone2 < SOIL_DRY_THRESHOLD && !pump2Status) {
    Serial.printf("Zone 2 Khô (%.1f%%) -> BẬT Bơm 2\n", lastSoilZone2);
    if (sendPumpCommand(2, true)) pump2Status = true;
  } 
  else if (lastSoilZone2 > SOIL_WET_THRESHOLD && pump2Status) {
    Serial.printf("Zone 2 Ẩm (%.1f%%) -> TẮT Bơm 2\n", lastSoilZone2);
    if (sendPumpCommand(2, false)) pump2Status = false;
  }
}

// ================= 11. CHECK CONNECTION =================
void checkConnection() {
  if (WiFi.status() == WL_CONNECTED) {
    lastWiFiCheckTime = millis();

    if (isOfflineMode) {
      Serial.println(">>> WIFI OK -> ONLINE MODE <<<");
      isOfflineMode = false;
      client.setServer(MQTT_SERVER, MQTT_PORT);
      client.setCallback(callback);
    }

    if (!client.connected()) {
      Serial.print("MQTT Connecting...");
      if (client.connect("ESP32_Gateway_S3", MQTT_TOKEN, "", GW_ATTR_TOPIC, 1, true, GW_STATUS_OFFLINE)) {
        Serial.println("OK!");
        // Mark gateway ONLINE (attributes) + provide telemetry key for dashboards
        client.publish(GW_ATTR_TOPIC, GW_STATUS_ONLINE, true);
        lastGwHeartbeat = 0; // publish heartbeat ASAP

        client.subscribe("v1/gateway/rpc");
      } else {
        Serial.println("FAIL");
        delay(2000);
      }
    }
    client.loop();

  } else {
    unsigned long now = millis();
    if (!isOfflineMode && (now - lastWiFiCheckTime > OFFLINE_TIMEOUT)) {
      Serial.println("!!! LOST WIFI -> OFFLINE MODE !!!");
      isOfflineMode = true;
      WiFi.disconnect();
      lastReconnectAttempt = now;
    }
    if (isOfflineMode && (now - lastReconnectAttempt > RECONNECT_INTERVAL)) {
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

      lastReconnectAttempt = now;
    }
  }
}

// ================= MODULE: KIỂM TRA SỨC KHỎE NODE =================
void checkNodeHealth() {
  if (isOfflineMode) return; 

  static unsigned long lastHealthCheck = 0;
  unsigned long now = millis();

  if (now - lastHealthCheck < HEALTH_CHECK_INTERVAL) return;
  lastHealthCheck = now;

  // 1. Check Zone 1
  if (isZone1Online && (now - lastSeenZone1 > NODE_TIMEOUT_MS)) {
    Serial.println("!!! CẢNH BÁO: Zone 1 mất kết nối !!!");
    isZone1Online = false;
    reportDeviceStatus("Zone_1", false);
  }

  // 2. Check Zone 2
  if (isZone2Online && (now - lastSeenZone2 > NODE_TIMEOUT_MS)) {
    Serial.println("!!! CẢNH BÁO: Zone 2 mất kết nối !!!");
    isZone2Online = false;
    reportDeviceStatus("Zone_2", false);
  }

  // 3. Check Central
  if (isCentralOnline && (now - lastSeenCentral > NODE_TIMEOUT_MS)) {
    Serial.println("!!! CẢNH BÁO: Central Station mất kết nối !!!");
    isCentralOnline = false;
    reportDeviceStatus("Central_Station", false);
    centralError = true; // Coi như lỗi để chặn lệnh bơm
  }
  if (!isCentralOnline && (pump1Status || pump2Status)) {
  Serial.println("!!! EMERGENCY: Central offline nhưng bơm đang ON → FORCE STOP !!!");
  // Không có Central = không biết flow = nguy hiểm
  pump1Status = false;
  pump2Status = false;
  }
}

// ================= 12. SETUP =================
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(115200);
  delay(1500);
  Serial.println("\n=== GATEWAY FINAL (FULL + FIXES) ===");

  // Set MAC
  esp_base_mac_addr_set(myBaseMac);
  WiFi.mode(WIFI_AP_STA);
  
  // Set Channel Wifi 
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) ESP.restart();
  esp_now_register_recv_cb(OnDataRecv);

  // Register Peer (Central Node)
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, centralNodeMac, 6);
  peerInfo.channel = WIFI_CHANNEL;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  // Init WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  // ---- Time sync (for daily volume by day) ----
  //UTC+7.
  configTzTime("ICT-7", "pool.ntp.org", "time.nist.gov");
lastWiFiCheckTime = millis();

  // Init MQTT
  client.setServer(MQTT_SERVER, MQTT_PORT);
  client.setCallback(callback);
  client.setBufferSize(2048); 
  client.setKeepAlive(15);
  client.setSocketTimeout(15);
  
  // Khởi tạo thời gian tránh báo lỗi ngay khi bật
  unsigned long now = millis();
  lastSeenZone1 = now; lastSeenZone2 = now; lastSeenCentral = now;
}

// ================= 13. LOOP =================
void loop() {
  checkConnection();

  if (isOfflineMode) {
    runOfflineLogic();
  } else {
    checkNodeHealth();
  }
  
  publishGatewayHeartbeat();

  delay(10);
}