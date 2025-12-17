#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_system.h>

// (Optional) disable brownout on some ESP32 boards
#if __has_include("soc/rtc_cntl_reg.h")
  #include "soc/rtc_cntl_reg.h"
  #include "soc/soc.h"
#endif

// ============================================================
// CENTRAL STATION (BỂ CHỨA / TRẠM BƠM) - 2 PUMPS + SIM MODE
//
// Mục tiêu:
// 1) Central Station ESP32 vẫn chạy thật, giao tiếp ESP-NOW về Gateway.
// 2) Khi CHƯA có linh kiện (flow sensor, phao, relay/bơm), ta bật SIM_MODE:
//    - tự mô phỏng flow/volume/mực nước và lỗi chạy khô
//    - vẫn nhận lệnh từ Gateway (cmd_id=1, pump_id=1/2, action=0/1)
//    - vẫn gửi báo cáo về Gateway (id=0) để Gateway -> CoreIoT như bình thường
//
// Tương thích Gateway hiện tại của bạn:
// - Central gửi struct_message {id, temp, hum, soil}
//   id=0: temp = total_flow_lpm (tổng 2 bơm)
//        hum  = total_volume_l  (tổng 2 bơm)
//        soil = pump_status     (0=all OFF, 1=any ON, 2=any ERROR)
// - Mở rộng (để phân biệt lỗi từng bơm + mực nước bể):
//   id=10: Pump1 -> temp=flow_lpm, hum=total_l, soil=status(0/1/2)
//   id=11: Pump2 -> temp=flow_lpm, hum=total_l, soil=status(0/1/2)
//   id=12: Tank  -> temp=level_pct, hum=level_l, soil=tank_low(0/1)
// ============================================================

// ----------------- PHẦN CỨNG -----------------
#define RELAY_1_PIN 4   // Bơm Zone 1
#define RELAY_2_PIN 5   // Bơm Zone 2

#define FLOW_1_PIN  6   // Flow sensor đường Zone 1 (YF-S201)
#define FLOW_2_PIN  7   // Flow sensor đường Zone 2 (YF-S201)

// Phao/nút giả lập: LOW = tác động (chập GND), HIGH = thả
// - SIM_MODE: dùng pin này làm nút tay (short press toggle pump1, long press toggle pump2, double press reset)
// - REAL MODE: có thể dùng làm phao mức thấp (cạn bể)
#define WATER_LEVEL_PIN 10

// ----------------- MẠNG -----------------
#define WIFI_CHANNEL 6

// ID packets (tách biệt để Gateway phân biệt Pump1/Pump2/Tank).
// Giữ ID_CENTRAL_SUM = 0 để tương thích gateway/dashboard hiện tại.
#define ID_CENTRAL_SUM 0
#define ID_PUMP1       10
#define ID_PUMP2       11
#define ID_TANK        12

uint8_t gatewayMac[] = {0x7C, 0xDF, 0xA1, 0x00, 0x00, 0x01};
uint8_t myMac[]      = {0x7C, 0xDF, 0xA1, 0x00, 0x00, 0x05};

// ----------------- GIAO THỨC (TƯƠNG THÍCH GATEWAY) -----------------
typedef struct {
  int   id;
  float temp;  // Central: total flow (L/min)
  float hum;   // Central: total volume (L)
  float soil;  // Central: status (0/1/2)
} struct_message;

typedef struct {
  int cmd_id;   // 1: điều khiển bơm
  int pump_id;  // 1 hoặc 2
  int action;   // 1: ON, 0: OFF
} struct_command;

// ----------------- CHẾ ĐỘ SIM -----------------
#define SIM_MODE 1   // 1 = mô phỏng sensor, 0 = đọc sensor thật

// Mô phỏng nâng cao: kịch bản lỗi/đặc tính thực tế
enum SimScenario : uint8_t { SIM_NORMAL=0, SIM_DRYRUN=1, SIM_CLOGGING=2, SIM_GLITCH=3, SIM_RELAY_STUCK=4, SIM_RANDOM=5 };
SimScenario simScenario = SIM_RANDOM;

// Chu kỳ cập nhật
static const uint32_t TICK_MS = 1000;

// Ngưỡng bảo vệ chạy khô
static const float MIN_FLOW_LPM = 1.0f;
static const uint32_t DRYRUN_DELAY_MS = 10000;

// Bể nước mô phỏng (dung tích)
static const float TANK_CAPACITY_L = 40.0f; // chỉnh theo bể thật (demo ok)

// Khi mực nước xuống thấp, flow sẽ tụt mạnh (hút khí)
static const float LOW_WATER_THRESHOLD_PCT = 8.0f;

// Xác suất glitch (mỗi tick)
static const float GLITCH_PROB = 0.05f;

// ----------------- FLOW SENSOR (REAL MODE) -----------------
volatile long pulseCount1 = 0;
volatile long pulseCount2 = 0;
static const float CALIBRATION_FACTOR = 7.5f; // YF-S201 thường ~7.5

// ----------------- TRẠNG THÁI BƠM -----------------
struct PumpState {
  bool on = false;
  bool error = false;         // lỗi chạy khô
  bool relayStuck = false;    // mô phỏng relay kẹt (OFF nhưng vẫn ON)
  uint32_t startTimeMs = 0;   // thời điểm bật (để check dry-run)
  float flowLpm = 0.0f;
  float totalL = 0.0f;
  uint32_t lowFlowAccumMs = 0; // tích lũy thời gian flow < MIN_FLOW_LPM
};

PumpState p1, p2;
float tankLevelPct = 80.0f; // % mực nước mô phỏng

// ----------------- BIẾN GỬI / NHẬN -----------------
struct_message txMsg;
struct_command rxCmd;

// ----------------- ISR FLOW SENSOR -----------------
void IRAM_ATTR pulseCounter1() { pulseCount1++; }
void IRAM_ATTR pulseCounter2() { pulseCount2++; }

// ============================================================
// TIỆN ÍCH
// ============================================================
static inline float clampf(float x, float lo, float hi) { return (x < lo) ? lo : (x > hi) ? hi : x; }

float randUniform(float a, float b) {
  return a + (b - a) * (float)random(0, 10001) / 10000.0f;
}

// ============================================================
// ĐIỀU KHIỂN RELAY (kể cả SIM_MODE vẫn set pin để debug)
// ============================================================
void applyRelayPins() {
  // Lưu ý: module relay có thể active LOW. Nếu relay của bạn active LOW, hãy đảo HIGH/LOW ở đây.
  digitalWrite(RELAY_1_PIN, p1.on ? HIGH : LOW);
  digitalWrite(RELAY_2_PIN, p2.on ? HIGH : LOW);
}

// ============================================================
// XỬ LÝ LỆNH TỪ GATEWAY (ESP-NOW)
// ============================================================
void setPump(PumpState &p, int pumpId, bool turnOn) {
  if (turnOn) {
    if (p.error) {
      Serial.printf("[CMD] Pump%d: đang lỗi -> từ chối bật (cần reset)\n", pumpId);
      return;
    }
    p.on = true;
    p.startTimeMs = millis();
    p.lowFlowAccumMs = 0;
    Serial.printf("[CMD] Pump%d: ON\n", pumpId);
  } else {
    // mô phỏng relay kẹt (chỉ tác dụng trong SIM_RELAY_STUCK hoặc SIM_RANDOM)
    if (p.relayStuck) {
      // 70% là kẹt không tắt
      if (random(0, 100) < 70) {
        Serial.printf("[CMD] Pump%d: OFF nhưng relay kẹt -> vẫn ON\n", pumpId);
        p.on = true;
      } else {
        p.on = false;
        Serial.printf("[CMD] Pump%d: OFF (thoát kẹt)\n", pumpId);
      }
    } else {
      p.on = false;
      Serial.printf("[CMD] Pump%d: OFF\n", pumpId);
    }
  }
  applyRelayPins();
}

// Reset lỗi + (tuỳ chọn) nạp lại bể để demo
void resetErrors(bool refillTank = true) {
  p1.error = false; p2.error = false;
  p1.lowFlowAccumMs = 0; p2.lowFlowAccumMs = 0;
  if (refillTank) tankLevelPct = 80.0f;
  Serial.println("[RESET] Cleared errors" + String(refillTank ? " + refill tank" : ""));
}

// Callback nhận ESP-NOW
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  if (len < (int)sizeof(struct_command)) {
    Serial.printf("[RX] Len=%d (too small)\n", len);
    return;
  }
  memcpy(&rxCmd, incomingData, sizeof(rxCmd));

  if (rxCmd.cmd_id != 1) {
    Serial.printf("[RX] cmd_id=%d (ignored)\n", rxCmd.cmd_id);
    return;
  }

  bool turnOn = (rxCmd.action == 1);

  if (rxCmd.pump_id == 1) setPump(p1, 1, turnOn);
  else if (rxCmd.pump_id == 2) setPump(p2, 2, turnOn);
  else Serial.printf("[RX] pump_id=%d (ignored)\n", rxCmd.pump_id);
}

// ============================================================
// MÔ PHỎNG SENSOR (SIM_MODE)
// ============================================================
void simMaybeUpdateScenario() {
  if (simScenario != SIM_RANDOM) return;

  // Mỗi ~20-40s đổi trạng thái mô phỏng một chút cho giống đời
  static uint32_t nextChangeMs = 0;
  uint32_t now = millis();
  if (now < nextChangeMs) return;
  nextChangeMs = now + (uint32_t)random(20000, 40001);

  // 0..99
  int r = random(0, 100);
  if (r < 60) simScenario = SIM_NORMAL;
  else if (r < 72) simScenario = SIM_CLOGGING;
  else if (r < 84) simScenario = SIM_GLITCH;
  else if (r < 92) simScenario = SIM_RELAY_STUCK;
  else simScenario = SIM_DRYRUN;

  Serial.printf("[SIM] Scenario -> %d\n", (int)simScenario);
}

float simFlowForPump(const PumpState &p, int pumpId, float nominal) {
  if (!p.on || p.error) return 0.0f;

  float base = nominal;

  // Mực nước thấp -> hút khí -> flow tụt mạnh
  if (tankLevelPct <= LOW_WATER_THRESHOLD_PCT) {
    base *= 0.05f;
  }

  // Scenario tác động
  if (simScenario == SIM_CLOGGING) {
    // Nghẹt ống: giảm flow (pump1 giảm mạnh hơn để dễ thấy)
    base *= (pumpId == 1) ? 0.65f : 0.75f;
  } else if (simScenario == SIM_DRYRUN) {
    // Ép chạy khô: flow gần 0 dù bơm đang ON
    base *= 0.02f;
  }

  // Thêm nhiễu nhẹ
  base += randUniform(-0.20f, 0.20f);
  base = max(0.0f, base);

  // Glitch cảm biến: 1 tick spike hoặc 0
  bool allowGlitch = (simScenario == SIM_GLITCH || simScenario == SIM_RANDOM);
  if (allowGlitch && randUniform(0.0f, 1.0f) < GLITCH_PROB) {
    if (random(0, 2) == 0) base = 0.0f;
    else base *= randUniform(1.8f, 3.0f);
  }

  return base;
}

void simUpdateOneTick(uint32_t dtMs) {
  simMaybeUpdateScenario();

  // Kịch bản relay kẹt
  if (simScenario == SIM_RELAY_STUCK) {
    p1.relayStuck = true;
    p2.relayStuck = true;
  } else if (simScenario == SIM_NORMAL || simScenario == SIM_DRYRUN || simScenario == SIM_CLOGGING || simScenario == SIM_GLITCH) {
    // thỉnh thoảng tắt relayStuck để demo "lúc kẹt lúc không"
    p1.relayStuck = false;
    p2.relayStuck = false;
  }

  // nominal flow (L/min) cho 2 bơm (demo)
  float nominal1 = 3.5f;
  float nominal2 = 3.0f;

  // Update flow
  p1.flowLpm = simFlowForPump(p1, 1, nominal1);
  p2.flowLpm = simFlowForPump(p2, 2, nominal2);

  // Update total + tank
  float dtMin = (float)dtMs / 60000.0f;
  p1.totalL += p1.flowLpm * dtMin;
  p2.totalL += p2.flowLpm * dtMin;

  float outL = (p1.flowLpm + p2.flowLpm) * dtMin;
  float deltaPct = (outL / TANK_CAPACITY_L) * 100.0f;
  tankLevelPct = clampf(tankLevelPct - deltaPct, 0.0f, 100.0f);

  // Dry-run protection (giống code thật)
  auto dryRunCheck = [&](PumpState &p, int pumpId) {
    if (!p.on || p.error) { p.lowFlowAccumMs = 0; return; }

    if (p.flowLpm < MIN_FLOW_LPM) p.lowFlowAccumMs += dtMs;
    else p.lowFlowAccumMs = 0;

    if ((millis() - p.startTimeMs) > DRYRUN_DELAY_MS && p.lowFlowAccumMs >= DRYRUN_DELAY_MS) {
      Serial.printf("!!! [PROTECT] Pump%d DRY-RUN -> ERROR + OFF\n", pumpId);
      p.error = true;
      p.on = false;
      p.flowLpm = 0.0f;
      applyRelayPins();
    }
  };

  dryRunCheck(p1, 1);
  dryRunCheck(p2, 2);
}

// ============================================================
// REAL MODE: đọc flow sensor thật (2 kênh) + bảo vệ
// ============================================================
void realUpdateOneTick(uint32_t dtMs) {
  // Tính flow trong khoảng dtMs (mặc định 1s)
  // Flow(L/min) = (pulses / calibration) * (1000/dtMs)
  // (vì calibration factor thường được tính theo Hz)
  long c1 = pulseCount1; pulseCount1 = 0;
  long c2 = pulseCount2; pulseCount2 = 0;

  float scale = 1000.0f / (float)dtMs;

  p1.flowLpm = (c1 * scale) / CALIBRATION_FACTOR;
  p2.flowLpm = (c2 * scale) / CALIBRATION_FACTOR;

  // Total volume
  float dtMin = (float)dtMs / 60000.0f;
  p1.totalL += p1.flowLpm * dtMin;
  p2.totalL += p2.flowLpm * dtMin;

  // Phao mức thấp (nếu có): LOW = cạn -> force flow thấp -> sẽ kích dry-run
  if (digitalRead(WATER_LEVEL_PIN) == LOW) {
    // cạn bể -> coi như flow gần 0
    p1.flowLpm *= 0.02f;
    p2.flowLpm *= 0.02f;
  }

  auto dryRunCheck = [&](PumpState &p, int pumpId) {
    if (!p.on || p.error) { p.lowFlowAccumMs = 0; return; }

    if (p.flowLpm < MIN_FLOW_LPM) p.lowFlowAccumMs += dtMs;
    else p.lowFlowAccumMs = 0;

    if ((millis() - p.startTimeMs) > DRYRUN_DELAY_MS && p.lowFlowAccumMs >= DRYRUN_DELAY_MS) {
      Serial.printf("!!! [PROTECT] Pump%d DRY-RUN -> ERROR + OFF\n", pumpId);
      p.error = true;
      p.on = false;
      p.flowLpm = 0.0f;
      applyRelayPins();
    }
  };

  dryRunCheck(p1, 1);
  dryRunCheck(p2, 2);
}

// ============================================================
// GỬI BÁO CÁO VỀ GATEWAY
// ============================================================
// Gửi 1 packet ESP-NOW theo format {id,temp,hum,soil}
static bool sendMsg(int id, float a, float b, float c) {
  txMsg.id   = id;
  txMsg.temp = a;
  txMsg.hum  = b;
  txMsg.soil = c;
  esp_err_t r = esp_now_send(gatewayMac, (uint8_t*)&txMsg, sizeof(txMsg));
  if (r != ESP_OK) Serial.println(">> [ESP-NOW] Send FAIL");
  return r == ESP_OK;
}

void sendReport() {
  // 1) Packet tổng (giữ tương thích với Gateway hiện tại)
  float totalFlow = p1.flowLpm + p2.flowLpm;
  float totalVol  = p1.totalL  + p2.totalL;

  // status: 2 nếu có lỗi, 1 nếu có bơm bật, 0 nếu tất cả tắt
  float statusAll = 0.0f;
  if (p1.error || p2.error) statusAll = 2.0f;
  else if (p1.on || p2.on)  statusAll = 1.0f;

  sendMsg(ID_CENTRAL_SUM, totalFlow, totalVol, statusAll);

  // 2) Packet Pump1 riêng
// Bitmask status (để phân biệt trạng thái/lỗi):
// bit0=ON, bit1=DRYRUN_ERROR, bit2=RELAY_STUCK
float st1 = (p1.on ? 1.0f : 0.0f) + (p1.error ? 2.0f : 0.0f) + (p1.relayStuck ? 4.0f : 0.0f);
sendMsg(ID_PUMP1, p1.flowLpm, p1.totalL, st1);

// 3) Packet Pump2 riêng
float st2 = (p2.on ? 1.0f : 0.0f) + (p2.error ? 2.0f : 0.0f) + (p2.relayStuck ? 4.0f : 0.0f);
sendMsg(ID_PUMP2, p2.flowLpm, p2.totalL, st2);

// 4) Packet Tank riêng: % + L + bitmask
// bit0 = tank_low(<20%), bit1 = low_level_switch (REAL MODE: WATER_LEVEL_PIN == LOW)
float tankL = (tankLevelPct / 100.0f) * TANK_CAPACITY_L;
float tankLowPct = (tankLevelPct < 20.0f) ? 1.0f : 0.0f; // ngưỡng demo, chỉnh tuỳ bạn
float tankLowSwitch = (digitalRead(WATER_LEVEL_PIN) == LOW) ? 2.0f : 0.0f;
float tankFlags = tankLowPct + tankLowSwitch;
sendMsg(ID_TANK, tankLevelPct, tankL, tankFlags);
}


// ============================================================
// NÚT TAY (SIM_MODE): short press toggle P1, long press toggle P2,
// double press reset errors + refill tank.
// ============================================================
void handleManualButton() {
  static bool lastState = HIGH;
  static uint32_t pressedAt = 0;
  static uint32_t lastReleaseAt = 0;
  static bool pendingSingle = false;

  bool s = digitalRead(WATER_LEVEL_PIN);

  // press
  if (lastState == HIGH && s == LOW) {
    pressedAt = millis();
  }

  // release
  if (lastState == LOW && s == HIGH) {
    uint32_t dur = millis() - pressedAt;

    // detect double click within 350ms
    if (millis() - lastReleaseAt < 350) {
      pendingSingle = false;
      resetErrors(true);
    } else {
      // queue single/long
      pendingSingle = true;

      if (dur > 1200) {
        // long press -> pump2
        setPump(p2, 2, !p2.on);
        pendingSingle = false;
      }
    }
    lastReleaseAt = millis();
  }

  // after delay, execute single click -> pump1
  if (pendingSingle && (millis() - lastReleaseAt > 350)) {
    setPump(p1, 1, !p1.on);
    pendingSingle = false;
  }

  lastState = s;
}

// ============================================================
// SERIAL COMMANDS (tuỳ chọn) để demo chủ động
// ------------------------------------------------------------
// - SCENARIO normal|dryrun|clogging|glitch|relay_stuck|random
// - REFILL <0..100>
// - RESET
// - P1 ON/OFF
// - P2 ON/OFF
// ============================================================
void handleSerialCommands() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  line.toLowerCase();

  auto startsWith = [&](const char* p) { return line.startsWith(p); };

  if (startsWith("scenario")) {
    if (line.indexOf("normal") >= 0) simScenario = SIM_NORMAL;
    else if (line.indexOf("dryrun") >= 0) simScenario = SIM_DRYRUN;
    else if (line.indexOf("clogging") >= 0) simScenario = SIM_CLOGGING;
    else if (line.indexOf("glitch") >= 0) simScenario = SIM_GLITCH;
    else if (line.indexOf("relay") >= 0) simScenario = SIM_RELAY_STUCK;
    else simScenario = SIM_RANDOM;
    Serial.printf("[SIM] Scenario set -> %d\n", (int)simScenario);
  } else if (startsWith("refill")) {
    float pct = line.substring(6).toFloat();
    tankLevelPct = clampf(pct, 0.0f, 100.0f);
    Serial.printf("[SIM] Tank refill -> %.1f%%\n", tankLevelPct);
  } else if (startsWith("reset")) {
    resetErrors(true);
  } else if (startsWith("p1")) {
    if (line.indexOf("on") >= 0) setPump(p1, 1, true);
    else if (line.indexOf("off") >= 0) setPump(p1, 1, false);
    else Serial.println("Use: P1 ON|OFF");
  } else if (startsWith("p2")) {
    if (line.indexOf("on") >= 0) setPump(p2, 2, true);
    else if (line.indexOf("off") >= 0) setPump(p2, 2, false);
    else Serial.println("Use: P2 ON|OFF");
  } else if (startsWith("status")) {
    Serial.printf("[STATUS] tank=%.1f%% | P1(on=%d err=%d flow=%.2f tot=%.2f) | P2(on=%d err=%d flow=%.2f tot=%.2f)\n",
      tankLevelPct,
      (int)p1.on, (int)p1.error, p1.flowLpm, p1.totalL,
      (int)p2.on, (int)p2.error, p2.flowLpm, p2.totalL
    );
  } else {
    Serial.println("Commands: SCENARIO ..., REFILL <pct>, RESET, P1 ON/OFF, P2 ON/OFF, STATUS");
  }
}

// ============================================================
// SETUP / LOOP
// ============================================================
void setup() {
#if __has_include("soc/rtc_cntl_reg.h")
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
#endif

  Serial.begin(115200);
  delay(800);
  Serial.println("\n=== CENTRAL STATION (2 Pumps) ===");
#if SIM_MODE
  Serial.println("[MODE] SIM_MODE=ON (no sensors required)");
#else
  Serial.println("[MODE] SIM_MODE=OFF (read real sensors)");
#endif

  randomSeed((uint32_t)esp_random());

  pinMode(RELAY_1_PIN, OUTPUT);
  pinMode(RELAY_2_PIN, OUTPUT);
  digitalWrite(RELAY_1_PIN, LOW);
  digitalWrite(RELAY_2_PIN, LOW);

  pinMode(WATER_LEVEL_PIN, INPUT_PULLUP);

#if !SIM_MODE
  pinMode(FLOW_1_PIN, INPUT_PULLUP);
  pinMode(FLOW_2_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLOW_1_PIN), pulseCounter1, FALLING);
  attachInterrupt(digitalPinToInterrupt(FLOW_2_PIN), pulseCounter2, FALLING);
#endif

  // ESP-NOW init
  esp_base_mac_addr_set(myMac);
  WiFi.mode(WIFI_STA);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] init FAILED");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, gatewayMac, 6);
  peerInfo.channel = WIFI_CHANNEL;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("[ESP-NOW] add peer FAILED");
  } else {
    Serial.println("[ESP-NOW] Ready");
  }
}

void loop() {
  static uint32_t lastTick = 0;
  uint32_t now = millis();

  // manual button + serial commands help demo even without cloud
#if SIM_MODE
  handleManualButton();
  handleSerialCommands();
#endif

  if (now - lastTick >= TICK_MS) {
    uint32_t dt = now - lastTick;
    lastTick = now;

#if SIM_MODE
    simUpdateOneTick(dt);
#else
    realUpdateOneTick(dt);
#endif

    // Print debug (1 dòng)
    Serial.printf("[Central] tank=%.1f%% | P1:%d err:%d flow:%.2f tot:%.2f | P2:%d err:%d flow:%.2f tot:%.2f\n",
      tankLevelPct,
      (int)p1.on, (int)p1.error, p1.flowLpm, p1.totalL,
      (int)p2.on, (int)p2.error, p2.flowLpm, p2.totalL
    );

    sendReport();
  }
}
