#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <math.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// ================= 1. CẤU HÌNH HỆ THỐNG =================
const char* WIFI_SSID       = "SJQKA";
const char* WIFI_PASSWORD   = "1236547890";

const char* MQTT_SERVER     = "app.coreiot.io";
const int   MQTT_PORT       = 1883;
const char* MQTT_TOKEN      = "UBXoN28TXw7FLwqP0ocy";

// ESPNOW/WiFi channel phải trùng nhau (AP của bạn nên ở kênh này)
#define WIFI_CHANNEL            6

// Cấu hình Logic Offline (tưới theo soil)
#define SOIL_DRY_THRESHOLD      30.0f  // < 30% -> BẬT BƠM
#define SOIL_WET_THRESHOLD      60.0f  // > 60% -> TẮT BƠM
#define OFFLINE_TIMEOUT         60000UL  // Mất WiFi > 60s -> OFFLINE MODE
#define RECONNECT_INTERVAL      30000UL  // OFFLINE: thử reconnect WiFi mỗi 30s

// Device ảo khí hậu chung
#define ENV_DEVICE_NAME         "Env_Area"
#define ENV_STALE_MS            120000UL  // 2 phút không có data -> coi như stale
#define ENV_PUB_INTERVAL_MS     2000UL    // giới hạn publish Env_Area (tránh spam)

// ================= 1.1. CENTRAL EXTENDED IDs (SIM) =================
// Giữ tương thích: Central vẫn gửi id=0 (tổng) như cũ.
#define ID_CENTRAL_SUM          0
#define ID_PUMP1                10
#define ID_PUMP2                11
#define ID_TANK                 12

// Bitmask cho Pump status (field incomingData.soil khi id=10/11)
#define PUMP_BIT_ON             0x01
#define PUMP_BIT_DRYRUN         0x02
#define PUMP_BIT_RELAY_STUCK    0x04

// Bitmask cho Tank flags (field incomingData.soil khi id=12)
#define TANK_FLAG_LOW_PCT       0x01
#define TANK_FLAG_LOW_SWITCH    0x02

// ================= 2. MAC ADDRESS =================
// Base MAC để set cho ESP32-S3 Gateway (ổn định MAC)
uint8_t myBaseMac[]      = {0x7C, 0xDF, 0xA1, 0x00, 0x00, 0x01}; // Gateway

// Central Station (Trạm bơm)
uint8_t centralNodeMac[] = {0x7C, 0xDF, 0xA1, 0x00, 0x00, 0x05}; // Central Station

// ================= 3. STRUCT ESPNOW =================
// Payload từ Zone/Central gửi về Gateway
// id=1/2: temp/hum/soil = T/H/Soil
// id=0  : temp/hum/soil = total_flow/total_volume/statusAll (0/1/2)
// id=10 : temp/hum/soil = pump1_flow/pump1_total/pump1_status_bits
// id=11 : temp/hum/soil = pump2_flow/pump2_total/pump2_status_bits
// id=12 : temp/hum/soil = tank_level_pct/tank_level_l/tank_flags_bits
typedef struct {
  int   id;
  float temp;
  float hum;
  float soil;
} struct_message;

struct_message incomingData;

// Lệnh gửi từ Gateway xuống Central Station để điều khiển bơm
typedef struct {
  int cmd_id;   // 1: pump control
  int pump_id;  // 1 hoặc 2
  int action;   // 1=ON, 0=OFF
} struct_command;

struct_command cmd;

// ================= 4. MQTT =================
WiFiClient espClient;
PubSubClient client(espClient);

// ================= 5. BIẾN TRẠNG THÁI ONLINE/OFFLINE =================
unsigned long lastWiFiCheckTime    = 0;
unsigned long lastReconnectAttempt = 0;

bool isOfflineMode = false;

// Cache soil cho offline logic
float lastSoilZone1 = 100.0f;
float lastSoilZone2 = 100.0f;

bool pump1Status = false;
bool pump2Status = false;

// Cờ lỗi tổng từ Central (dry-run / error)
bool centralError = false;
bool centralSumError = false;
bool pump1Error = false;
bool pump2Error = false;

// Cache Central extended data (để debug / có thể dùng rule offline về sau)
float pump1Flow = 0.0f, pump2Flow = 0.0f;
float pump1Total = 0.0f, pump2Total = 0.0f;
int   pump1StatusBits = 0, pump2StatusBits = 0;

float tankLevelPct = NAN;
float tankLevelL   = NAN;
int   tankFlagsBits = 0;

// Cache T/H cho Env_Area
float lastTempZ1 = NAN, lastHumZ1 = NAN;
float lastTempZ2 = NAN, lastHumZ2 = NAN;
unsigned long lastEnvTsZ1 = 0, lastEnvTsZ2 = 0;
unsigned long lastEnvPublish = 0;

// ================= 6. HELPERS =================
bool getEnvAvg(float &tAvg, float &hAvg) {
  unsigned long now = millis();

  bool z1ok = (lastEnvTsZ1 != 0) &&
              (now - lastEnvTsZ1 <= ENV_STALE_MS) &&
              !isnan(lastTempZ1) && !isnan(lastHumZ1);

  bool z2ok = (lastEnvTsZ2 != 0) &&
              (now - lastEnvTsZ2 <= ENV_STALE_MS) &&
              !isnan(lastTempZ2) && !isnan(lastHumZ2);

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

  DynamicJsonDocument doc(256);
  JsonArray arr = doc.createNestedArray(ENV_DEVICE_NAME);
  JsonObject obj = arr.createNestedObject();
  obj["temperature"] = tAvg;
  obj["humidity"]    = hAvg;

  String output;
  serializeJson(doc, output);
  client.publish("v1/gateway/telemetry", output.c_str());

  lastEnvPublish = now;
}

// ================= 7. ESPNOW SEND COMMAND =================
void sendPumpCommand(int pumpId, bool turnOn) {
  // Khi Central đang lỗi: CHẶN lệnh BẬT (ON) nhưng vẫn cho TẮT (OFF) để an toàn
  if (centralError && turnOn) {
    Serial.println("!!! CENTRAL ERROR -> CHẶN LỆNH BẬT (ON), vẫn cho phép TẮT (OFF) !!!");
    return;
  }

  cmd.cmd_id  = 1;
  cmd.pump_id = pumpId;
  cmd.action  = turnOn ? 1 : 0;

  esp_err_t result = esp_now_send(centralNodeMac, (uint8_t *)&cmd, sizeof(cmd));
  if (result == ESP_OK) {
    Serial.printf("[CMD] Pump%d -> %s\n", pumpId, turnOn ? "ON" : "OFF");
  } else {
    Serial.printf("[CMD] Gửi lệnh Pump%d lỗi!\n", pumpId);
  }
}

// ================= 8. ESPNOW RECV CALLBACK =================
void OnDataRecv(const uint8_t * mac, const uint8_t *incoming, int len) {
  if (len != sizeof(struct_message)) {
    Serial.printf("Nhận gói lạ len=%d\n", len);
    return;
  }

  memcpy(&incomingData, incoming, sizeof(incomingData));

  // --- Cập nhật cache cho offline + env avg ---
  if (incomingData.id == 1) {
    lastSoilZone1 = incomingData.soil;

    lastTempZ1 = incomingData.temp;
    lastHumZ1  = incomingData.hum;
    lastEnvTsZ1 = millis();

  } else if (incomingData.id == 2) {
    lastSoilZone2 = incomingData.soil;

    lastTempZ2 = incomingData.temp;
    lastHumZ2  = incomingData.hum;
    lastEnvTsZ2 = millis();

  } else if (incomingData.id == ID_CENTRAL_SUM) {
    // statusAll 0/1/2 (giữ tương thích cũ)
    centralSumError = ((int)incomingData.soil == 2);

  } else if (incomingData.id == ID_PUMP1) {
    pump1Flow = incomingData.temp;
    pump1Total = incomingData.hum;
    pump1StatusBits = (int)incomingData.soil;
    pump1Error = ((pump1StatusBits & PUMP_BIT_DRYRUN) != 0);

  } else if (incomingData.id == ID_PUMP2) {
    pump2Flow = incomingData.temp;
    pump2Total = incomingData.hum;
    pump2StatusBits = (int)incomingData.soil;
    pump2Error = ((pump2StatusBits & PUMP_BIT_DRYRUN) != 0);

  } else if (incomingData.id == ID_TANK) {
    tankLevelPct = incomingData.temp;
    tankLevelL   = incomingData.hum;
    tankFlagsBits = (int)incomingData.soil;
  }

  // Tổng hợp lỗi
  centralError = (centralSumError || pump1Error || pump2Error);

  // --- Online publish lên CoreIoT ---
  if (!isOfflineMode && client.connected()) {
    DynamicJsonDocument doc(768);

    String deviceName;

    // Central Station: id=0/10/11/12
    if (incomingData.id == ID_CENTRAL_SUM ||
        incomingData.id == ID_PUMP1 ||
        incomingData.id == ID_PUMP2 ||
        incomingData.id == ID_TANK) {

      deviceName = "Central_Station";
      JsonArray arr = doc.createNestedArray(deviceName);
      JsonObject obj = arr.createNestedObject();

      if (incomingData.id == ID_CENTRAL_SUM) {
        obj["flow_rate"]    = incomingData.temp;       // tổng (cũ)
        obj["total_volume"] = incomingData.hum;        // tổng (cũ)
        obj["pump_status"]  = (int)incomingData.soil;  // 0/1/2 (cũ)
      }
      else if (incomingData.id == ID_PUMP1) {
        obj["pump1_flow_rate"]    = incomingData.temp;
        obj["pump1_total_volume"] = incomingData.hum;
        obj["pump1_status_bits"]  = (int)incomingData.soil;
        obj["pump1_on"]           = (((int)incomingData.soil & PUMP_BIT_ON) != 0);
        obj["pump1_error"]        = (((int)incomingData.soil & PUMP_BIT_DRYRUN) != 0);
        obj["pump1_relay_stuck"]  = (((int)incomingData.soil & PUMP_BIT_RELAY_STUCK) != 0);
      }
      else if (incomingData.id == ID_PUMP2) {
        obj["pump2_flow_rate"]    = incomingData.temp;
        obj["pump2_total_volume"] = incomingData.hum;
        obj["pump2_status_bits"]  = (int)incomingData.soil;
        obj["pump2_on"]           = (((int)incomingData.soil & PUMP_BIT_ON) != 0);
        obj["pump2_error"]        = (((int)incomingData.soil & PUMP_BIT_DRYRUN) != 0);
        obj["pump2_relay_stuck"]  = (((int)incomingData.soil & PUMP_BIT_RELAY_STUCK) != 0);
      }
      else if (incomingData.id == ID_TANK) {
        obj["tank_level_pct"] = incomingData.temp;
        obj["tank_level_l"]   = incomingData.hum;
        obj["tank_flags_bits"]= (int)incomingData.soil;
        obj["tank_low_pct"]   = (((int)incomingData.soil & TANK_FLAG_LOW_PCT) != 0);
        obj["tank_low_switch"]= (((int)incomingData.soil & TANK_FLAG_LOW_SWITCH) != 0);
      }
    }
    // Zones (id=1/2)
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

    // Nếu là data zone thì publish thêm Env_Area (T/H trung bình)
    if (incomingData.id == 1 || incomingData.id == 2) {
      publishEnvAvgIfReady();
    }
  }
}

// ================= 9. MQTT RPC CALLBACK =================
// Hỗ trợ 2 format:
// (1) {"method":"setPump1","params":true}
// (2) {"device":"Central_Station","data":{"id":xxx,"method":"setPump1","params":true}}
void callback(char* topic, byte* payload, unsigned int length) {
  DynamicJsonDocument doc(1024);

  DeserializationError err = deserializeJson(doc, payload, length);
  if (err) {
    Serial.print("RPC JSON parse lỗi: ");
    Serial.println(err.c_str());
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

  if (!valid) {
    Serial.println("RPC format không hợp lệ!");
    return;
  }

  int pumpId = 0;
  if (method == "setPump1") pumpId = 1;
  else if (method == "setPump2") pumpId = 2;

  if (pumpId == 0) {
    Serial.printf("RPC method không hỗ trợ: %s\n", method.c_str());
    return;
  }

  // Gửi lệnh xuống Central
  sendPumpCommand(pumpId, params);

  // Cập nhật trạng thái local (tránh spam)
  if (pumpId == 1) pump1Status = params;
  if (pumpId == 2) pump2Status = params;
}

// ================= 10. OFFLINE LOGIC =================
void runOfflineLogic() {
  static unsigned long lastOfflineLoop = 0;
  if (millis() - lastOfflineLoop < 5000) return;
  lastOfflineLoop = millis();

  Serial.println(">>> [OFFLINE MODE] Checking Soil Logic... <<<");

  if (centralError) {
    Serial.println("!!! Central Station báo LỖI -> Dừng mọi lệnh (OFFLINE AUTO) !!!");
    return;
  }

  // Zone 1
  if (lastSoilZone1 < SOIL_DRY_THRESHOLD && !pump1Status) {
    sendPumpCommand(1, true);
    pump1Status = true;
  } else if (lastSoilZone1 > SOIL_WET_THRESHOLD && pump1Status) {
    sendPumpCommand(1, false);
    pump1Status = false;
  }

  // Zone 2
  if (lastSoilZone2 < SOIL_DRY_THRESHOLD && !pump2Status) {
    sendPumpCommand(2, true);
    pump2Status = true;
  } else if (lastSoilZone2 > SOIL_WET_THRESHOLD && pump2Status) {
    sendPumpCommand(2, false);
    pump2Status = false;
  }
}

// ================= 11. CONNECTION MANAGEMENT =================
void checkConnection() {
  if (WiFi.status() == WL_CONNECTED) {
    lastWiFiCheckTime = millis();

    if (isOfflineMode) {
      Serial.println(">>> CÓ WIFI LẠI -> ONLINE MODE <<<");
      isOfflineMode = false;

      // Re-init MQTT
      client.setServer(MQTT_SERVER, MQTT_PORT);
      client.setCallback(callback);
    }

    if (!client.connected()) {
      Serial.print("MQTT Connecting...");
      if (client.connect("ESP32_Gateway_S3", MQTT_TOKEN, NULL)) {
        Serial.println("OK!");
        client.subscribe("v1/gateway/rpc");
      } else {
        Serial.print("Fail rc=");
        Serial.println(client.state());
        delay(2000);
      }
    }

    client.loop();

  } else {
    unsigned long now = millis();

    // Nếu mất wifi quá lâu -> OFFLINE MODE
    if (!isOfflineMode && (now - lastWiFiCheckTime > OFFLINE_TIMEOUT)) {
      Serial.println("!!! MẤT WIFI > 60s -> OFFLINE MODE !!!");
      isOfflineMode = true;
      WiFi.disconnect();
      lastReconnectAttempt = now;
    }

    // OFFLINE: thử reconnect định kỳ
    if (isOfflineMode && (now - lastReconnectAttempt > RECONNECT_INTERVAL)) {
      Serial.println(">>> OFFLINE: Thử reconnect WiFi... <<<");
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      lastReconnectAttempt = now;
    }
  }
}

// ================= 12. SETUP =================
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);
  delay(1500);
  Serial.println("\n=== GATEWAY ULTIMATE (ONLINE/OFFLINE + ENV AVG + CENTRAL EXT) ===");

  // Set base MAC cố định cho Gateway
  esp_base_mac_addr_set(myBaseMac);

  // WiFi STA
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  // Set channel cho ESPNOW (đảm bảo AP cùng channel)
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);

  // Add Central Station peer (để gửi lệnh bơm)
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, centralNodeMac, 6);
  peerInfo.channel = WIFI_CHANNEL;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Thêm peer Central Station thất bại!");
  }

  // WiFi begin
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  lastWiFiCheckTime = millis();

  // MQTT init
  client.setServer(MQTT_SERVER, MQTT_PORT);
  client.setCallback(callback);
  client.setBufferSize(1024);
}

// ================= 13. LOOP =================
void loop() {
  checkConnection();

  if (isOfflineMode) {
    runOfflineLogic();
  }

  delay(10);
}
