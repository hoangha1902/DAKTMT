#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ================= 1. CẤU HÌNH HỆ THỐNG =================
const char* WIFI_SSID       = "SJQKA";         
const char* WIFI_PASSWORD   = "1236547890";    

const char* MQTT_SERVER     = "app.coreiot.io"; 
const int   MQTT_PORT       = 1883;
const char* MQTT_TOKEN      = "UBXoN28TXw7FLwqP0ocy";

// Cấu hình Logic Offline
#define SOIL_DRY_THRESHOLD  30.0  // < 30% -> BẬT BƠM
#define SOIL_WET_THRESHOLD  60.0  // > 60% -> TẮT BƠM
#define OFFLINE_TIMEOUT     60000 // 60 giây mất mạng -> Vào chế độ Offline

// Địa chỉ MAC
uint8_t myBaseMac[] = {0x7C, 0xDF, 0xA1, 0x00, 0x00, 0x01}; // Gateway
uint8_t centralNodeMac[] = {0x7C, 0xDF, 0xA1, 0x00, 0x00, 0x05}; // Central Station

// ================= 2. BIẾN TOÀN CỤC =================
WiFiClient espClient;
PubSubClient client(espClient);

// Biến trạng thái
unsigned long lastWiFiCheckTime = 0;
bool isOfflineMode = false;
bool pump1Status = false;
bool pump2Status = false;
bool centralError = false; // Cờ lỗi từ Central Station

// "Trí nhớ tạm" (Cache) cho chế độ Offline
float lastSoilZone1 = 100.0; 
float lastSoilZone2 = 100.0;

// Struct Dữ Liệu
typedef struct {
  int id;       
  float temp;   // Zone: Temp | Central: Flow Rate
  float hum;    // Zone: Hum  | Central: Total Vol
  float soil;   // Zone: Soil | Central: Status (0=OFF, 1=ON, 2=ERR)
} struct_message;
struct_message incomingData;

typedef struct {
  int cmd_id;    // 1: Lệnh Bơm
  int pump_id;   // 1 hoặc 2
  int action;    // 1: Bật, 0: Tắt
} struct_command;
struct_command cmdToSend;

// ================= 3. HÀM GỬI LỆNH XUỐNG CENTRAL =================
void sendPumpCommand(int pumpId, bool turnOn) {
  cmdToSend.cmd_id = 1;
  cmdToSend.pump_id = pumpId;
  cmdToSend.action = turnOn ? 1 : 0;

  esp_err_t result = esp_now_send(centralNodeMac, (uint8_t *) &cmdToSend, sizeof(cmdToSend));
  
  if (result == ESP_OK) {
    Serial.printf("-> [CMD] Gửi lệnh Bơm %d: %s\n", pumpId, turnOn ? "ON" : "OFF");
  } else {
    Serial.println("-> [CMD] Gửi thất bại (Check Central Node)");
  }
}

// ================= 4. LOGIC OFFLINE MODE (TỰ ĐỘNG) =================
void runOfflineLogic() {
  static unsigned long lastOfflineLoop = 0;
  
  // Chạy mỗi 5 giây
  if (millis() - lastOfflineLoop < 5000) return;
  lastOfflineLoop = millis();

  Serial.println(">>> [OFFLINE MODE] Checking Logic... <<<");
  
  if (centralError) {
    Serial.println("!!! Central Station Lỗi -> Dừng mọi lệnh !!!");
    return;
  }

  // Logic Zone 1
  if (lastSoilZone1 < SOIL_DRY_THRESHOLD && !pump1Status) {
    Serial.printf("Z1 Khô (%.1f%%) -> Bật Bơm 1\n", lastSoilZone1);
    sendPumpCommand(1, true);
    pump1Status = true;
  } 
  else if (lastSoilZone1 > SOIL_WET_THRESHOLD && pump1Status) {
    Serial.printf("Z1 Ướt (%.1f%%) -> Tắt Bơm 1\n", lastSoilZone1);
    sendPumpCommand(1, false);
    pump1Status = false;
  }

  // Logic Zone 2
  if (lastSoilZone2 < SOIL_DRY_THRESHOLD && !pump2Status) {
    Serial.printf("Z2 Khô (%.1f%%) -> Bật Bơm 2\n", lastSoilZone2);
    sendPumpCommand(2, true);
    pump2Status = true;
  } 
  else if (lastSoilZone2 > SOIL_WET_THRESHOLD && pump2Status) {
    Serial.printf("Z2 Ướt (%.1f%%) -> Tắt Bơm 2\n", lastSoilZone2);
    sendPumpCommand(2, false);
    pump2Status = false;
  }
}

// ================= 5. HÀM NHẬN DATA (ESP-NOW) =================
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingDataPtr, int len) {
  memcpy(&incomingData, incomingDataPtr, sizeof(incomingData));

  // --- CẬP NHẬT CACHE ---
  if (incomingData.id == 1) lastSoilZone1 = incomingData.soil;
  else if (incomingData.id == 2) lastSoilZone2 = incomingData.soil;
  else if (incomingData.id == 0) {
    centralError = ((int)incomingData.soil == 2); // Status=2 là Lỗi
  }

  // --- GỬI LÊN CLOUD (CHỈ KHI ONLINE) ---
  if (!isOfflineMode && client.connected()) {
    DynamicJsonDocument doc(1024);
    String deviceName;
    String output;

    if (incomingData.id == 0) {
      deviceName = "Central_Station";
      JsonArray arr = doc.createNestedArray(deviceName);
      JsonObject obj = arr.createNestedObject();
      obj["flow_rate"]    = incomingData.temp;
      obj["total_volume"] = incomingData.hum;
      obj["pump_status"]  = (int)incomingData.soil;
    } else {
      deviceName = "Zone_" + String(incomingData.id);
      JsonArray arr = doc.createNestedArray(deviceName);
      JsonObject obj = arr.createNestedObject();
      obj["temperature"]   = incomingData.temp;
      obj["humidity"]      = incomingData.hum;
      obj["soil_moisture"] = incomingData.soil;
    }
    serializeJson(doc, output);
    client.publish("v1/gateway/telemetry", output.c_str());
  }
}

// ================= 6. HÀM RPC CALLBACK (TỪ CLOUD) =================
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print(">> [RPC] Nhận lệnh: ");
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.println(message);

  DynamicJsonDocument doc(1024);
  deserializeJson(doc, message);

  String method;
  bool params;

  // --- BÓC TÁCH JSON THÔNG MINH ---
  if (doc.containsKey("method")) {
    method = doc["method"].as<String>();
    params = doc["params"];
  } else if (doc.containsKey("device") && doc["device"] == "Central_Station") {
    method = doc["data"]["method"].as<String>();
    params = doc["data"]["params"];
  } else {
    Serial.println("-> Bỏ qua lệnh (Sai định dạng/đích đến)");
    return;
  }

  // --- XỬ LÝ & GỬI ---
  bool valid = false;
  int pumpId = 0;

  if (method == "setPump1") { pumpId = 1; valid = true; }
  else if (method == "setPump2") { pumpId = 2; valid = true; }

  if (valid) {
    sendPumpCommand(pumpId, params);
    // Cập nhật trạng thái local để đồng bộ
    if (pumpId == 1) pump1Status = params;
    else pump2Status = params;
  }
}

// ================= 7. QUẢN LÝ KẾT NỐI (AUTO SWITCH) =================
void checkConnection() {
  if (WiFi.status() == WL_CONNECTED) {
    lastWiFiCheckTime = millis();
    
    if (isOfflineMode) {
      Serial.println(">>> CÓ MẠNG LẠI -> ONLINE MODE <<<");
      isOfflineMode = false;
      // Re-init MQTT khi có mạng lại
      client.setServer(MQTT_SERVER, MQTT_PORT);
      client.setCallback(callback);
    }

    if (!client.connected()) {
      Serial.print("MQTT Connecting...");
      if (client.connect("ESP32_Gateway_S3", MQTT_TOKEN, NULL)) {
        Serial.println("OK!");
        client.subscribe("v1/gateway/rpc"); 
      } else {
        Serial.print("Fail rc="); Serial.println(client.state());
        delay(2000);
      }
    }
    client.loop();

  } else {
    // Mất mạng > 60s -> Offline Mode
    if (!isOfflineMode && (millis() - lastWiFiCheckTime > OFFLINE_TIMEOUT)) {
      Serial.println("!!! MẤT WIFI > 60s -> OFFLINE MODE !!!");
      isOfflineMode = true;
      WiFi.disconnect(); 
    }
  }
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n=== GATEWAY ULTIMATE (ONLINE/OFFLINE) ===");

  esp_base_mac_addr_set(myBaseMac);

  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  lastWiFiCheckTime = millis();

  if (esp_now_init() != ESP_OK) ESP.restart();
  esp_now_register_recv_cb(OnDataRecv);

  // Add Peer: Central Station
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, centralNodeMac, 6);
  peerInfo.channel = 6; 
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  // MQTT Init
  client.setServer(MQTT_SERVER, MQTT_PORT);
  client.setCallback(callback);
  client.setBufferSize(1024);
}

// ================= LOOP =================
void loop() {
  checkConnection();

  if (isOfflineMode) {
    runOfflineLogic();
  }
}