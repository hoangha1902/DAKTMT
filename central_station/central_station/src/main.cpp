/**
 * Central Station - Smart Irrigation
 * -------------------------------------------------
 * Vai trò:
 *  - Nhận lệnh từ Gateway qua ESP-NOW: bật/tắt Pump 1/2
 *  - Đọc cảm biến lưu lượng (Flow sensor) cho Pump 1/2
 *  - Đọc mức nước bể (tank level) (analog % hoặc công tắc báo cạn)
 *  - Áp dụng fail-safe: bể cạn / dry-run / relay-stuck (ước lượng)
 *  - Gửi telemetry trạng thái về Gateway qua ESP-NOW theo format tương thích main_gateway.cpp
 
 * GÓI TIN GỬI VỀ GATEWAY (struct_message):
 *  - ID_CENTRAL_SUM (0): temp=tank_level_pct, hum=pump_status_bits, soil=error_code (0 ok, 2 lỗi)
 *  - ID_PUMP1 (10): temp=pump1_flow_lpm, hum=pump1_total_l, soil=pump1_status_bits
 *  - ID_PUMP2 (11): temp=pump2_flow_lpm, hum=pump2_total_l, soil=pump2_status_bits
 *  - ID_TANK  (12): temp=tank_level_pct, hum=tank_level_l, soil=tank_flags_bits

 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Preferences.h>

// ======================================================
// 1) CẤU HÌNH 
// ======================================================


#ifndef WIFI_CHANNEL
#define WIFI_CHANNEL 11
#endif

// --- MAC địa chỉ của CENTRAL và GATEWAY ---
static uint8_t CENTRAL_BASE_MAC[6] = {0x7C, 0xDF, 0xA1, 0x00, 0x00, 0x05};
static uint8_t GATEWAY_MAC[6]      = {0x7C, 0xDF, 0xA1, 0x00, 0x00, 0x01};

// --- Relay pins (điều khiển bơm) ---
static constexpr int RELAY_1_PIN = 26;
static constexpr int RELAY_2_PIN = 27;

// Relay active level: nhiều module relay active-LOW
static constexpr bool RELAY_ACTIVE_LOW = true;

// --- Flow sensor pins (ngắt) ---
static constexpr int FLOW_1_PIN = 34; // GPIO34/35 chỉ input
static constexpr int FLOW_2_PIN = 35;

// --- Tank level ---
// Mode 1: Analog level (%) từ cảm biến mức nước (0..4095)
// Mode 2: Float switch báo cạn (digital)
enum TankLevelMode : uint8_t { TANK_LEVEL_ANALOG = 0, TANK_LEVEL_FLOAT_SWITCH = 1 };
static constexpr TankLevelMode TANK_MODE = TANK_LEVEL_ANALOG;

// (A) Analog tank level
static constexpr int TANK_LEVEL_ADC_PIN = 32;     // nếu dùng analog
static constexpr int TANK_ADC_MIN = 800;          // calib: giá trị ADC khi bể gần cạn
static constexpr int TANK_ADC_MAX = 3200;         // calib: giá trị ADC khi bể đầy
static constexpr float TANK_CAPACITY_L = 4500.0f; // 4000-5000L

// (B) Float switch báo cạn (active LOW, INPUT_PULLUP)
static constexpr int TANK_LOW_SWITCH_PIN = 25;    // nếu dùng float switch

// ======================================================
// 2) CẤU HÌNH LOGIC AN TOÀN (FAIL-SAFE)
// ======================================================

// Flow sensor calibration: pulses per liter (tuỳ loại YF-S201/YF-S402...)
// Nếu không chắc: để 450.0 rồi đo thực tế (đổ 10L xem total tăng bao nhiêu).
static constexpr float PULSES_PER_LITER = 450.0f;

// Nếu bơm ON nhưng flow < MIN_FLOW_LPM kéo dài -> dry-run (bơm chạy khan / mất nước / kẹt)
static constexpr float MIN_FLOW_LPM = 0.3f;
static constexpr uint32_t DRYRUN_GRACE_MS   = 3000;   // cho phép vài giây đầu chưa có nước
static constexpr uint32_t DRYRUN_TIMEOUT_MS = 8000;   // quá thời gian này mà flow vẫn thấp -> lỗi

// Nếu bơm OFF nhưng flow vẫn cao kéo dài -> nghi relay bị dính (hoặc van kẹt)
static constexpr float RELAY_STUCK_FLOW_LPM = 0.5f;
static constexpr uint32_t RELAY_STUCK_TIMEOUT_MS = 5000;

// Ngưỡng bể cạn theo % (khi dùng analog)
static constexpr float TANK_LOW_PCT_THRESHOLD = 8.0f;

// Chu kỳ xử lý & gửi report
static constexpr uint32_t TICK_MS = 1000;          // tính toán 1s/lần
static constexpr uint32_t REPORT_MS = 2000;        // gửi report 2s/lần
static constexpr uint32_t SAVE_MS = 60000;         // lưu NVS 60s/lần để tránh mòn flash

// ======================================================
// 3) ĐỊNH NGHĨA ID / BIT
// ======================================================
#define ID_CENTRAL_SUM          0
#define ID_PUMP1                10
#define ID_PUMP2                11
#define ID_TANK                 12

#define PUMP_BIT_ON             0x01
#define PUMP_BIT_DRYRUN         0x02
#define PUMP_BIT_RELAY_STUCK    0x04

#define TANK_FLAG_LOW_PCT       0x01
#define TANK_FLAG_LOW_SWITCH    0x02

// ======================================================
// 4) STRUCT GIAO TIẾP ESPNOW
// ======================================================
typedef struct {
  int   id;
  float temp;
  float hum;
  float soil;
} struct_message;

typedef struct {
  int cmd_id;
  int pump_id;
  int action; // 1=ON, 0=OFF
} struct_command;

// ======================================================
// 5) TRẠNG THÁI HỆ THỐNG
// ======================================================
static Preferences prefs;

static volatile uint32_t pulseCount1 = 0;
static volatile uint32_t pulseCount2 = 0;

struct PumpState {
  bool desiredOn = false;
  bool relayOn   = false;

  bool dryRun = false;
  bool relayStuck = false;

  uint32_t onSinceMs = 0;
  uint32_t flowBadSinceMs = 0;
  uint32_t flowWhileOffSinceMs = 0;

  float flowLpm = 0.0f;
  float totalL  = 0.0f; // tổng tích luỹ
};

static PumpState p1, p2;

static float tankLevelPct = 0.0f;
static float tankLevelL   = 0.0f;
static bool  tankLowSwitch = false;
static uint32_t tankFlagsBits = 0;

static bool centralHasError = false;

// ======================================================
// 6) ISR - FLOW SENSOR
// ======================================================
void IRAM_ATTR pulseCounter1() { pulseCount1++; }
void IRAM_ATTR pulseCounter2() { pulseCount2++; }

// ======================================================
// 7) HÀM PHỤ TRỢ
// ======================================================
static inline void relayWrite(int pin, bool on) {
  if (RELAY_ACTIVE_LOW) {
    digitalWrite(pin, on ? LOW : HIGH);
  } else {
    digitalWrite(pin, on ? HIGH : LOW);
  }
}

static inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static void loadPersistent() {
  prefs.begin("central", true);
  p1.totalL = prefs.getFloat("p1_totalL", 0.0f);
  p2.totalL = prefs.getFloat("p2_totalL", 0.0f);
  prefs.end();
}

static void savePersistent() {
  prefs.begin("central", false);
  prefs.putFloat("p1_totalL", p1.totalL);
  prefs.putFloat("p2_totalL", p2.totalL);
  prefs.end();
}

static void updateTankLevel() {
  tankFlagsBits = 0;

  // Float switch báo cạn (nếu có)
  if (TANK_MODE == TANK_LEVEL_FLOAT_SWITCH) {
    tankLowSwitch = (digitalRead(TANK_LOW_SWITCH_PIN) == LOW); // active low
    if (tankLowSwitch) tankFlagsBits |= TANK_FLAG_LOW_SWITCH;

    // Ở đây set 0% khi cạn, 80% khi OK (giả định)..
    tankLevelPct = tankLowSwitch ? 0.0f : 80.0f;
    tankLevelL   = (tankLevelPct / 100.0f) * TANK_CAPACITY_L;
    return;
  }

  // Analog % (khuyến nghị nếu muốn hiển thị L/PCT "thật")
  int raw = analogRead(TANK_LEVEL_ADC_PIN);
  float pct = 0.0f;

  if (TANK_ADC_MAX <= TANK_ADC_MIN) {
    // tránh cấu hình sai
    pct = 0.0f;
  } else {
    pct = 100.0f * (float)(raw - TANK_ADC_MIN) / (float)(TANK_ADC_MAX - TANK_ADC_MIN);
  }

  tankLevelPct = clampf(pct, 0.0f, 100.0f);
  tankLevelL   = (tankLevelPct / 100.0f) * TANK_CAPACITY_L;

  if (tankLevelPct <= TANK_LOW_PCT_THRESHOLD) tankFlagsBits |= TANK_FLAG_LOW_PCT;

  // Nếu  vẫn có float switch kèm theo (optional), bật dòng dưới:
  // tankLowSwitch = (digitalRead(TANK_LOW_SWITCH_PIN) == LOW);
  // if (tankLowSwitch) tankFlagsBits |= TANK_FLAG_LOW_SWITCH;
}

static void updateFlowAndTotals(uint32_t dtMs) {
  // Copy & clear pulse counts atomically
  uint32_t c1, c2;
  noInterrupts();
  c1 = pulseCount1; pulseCount1 = 0;
  c2 = pulseCount2; pulseCount2 = 0;
  interrupts();

  const float dtMin = (dtMs > 0) ? ((float)dtMs / 60000.0f) : 1.0f / 60.0f;

  // liters during dt = pulses / pulses_per_liter
  const float dL1 = (float)c1 / PULSES_PER_LITER;
  const float dL2 = (float)c2 / PULSES_PER_LITER;

  // flow (L/min) = delta liters / delta minutes
  p1.flowLpm = (dtMin > 0) ? (dL1 / dtMin) : 0.0f;
  p2.flowLpm = (dtMin > 0) ? (dL2 / dtMin) : 0.0f;

  // total accumulate only when pump is ON (tuỳ logic hệ thống)
  // Nếu flow meter nằm trên đường nước và vẫn có thể chảy khi OFF, vẫn nên cộng dL.
  // Ở đây cộng luôn để phản ánh đúng lượng nước đi qua.
  p1.totalL += dL1;
  p2.totalL += dL2;
}

static void applyRelay(PumpState &p, int relayPin, uint32_t nowMs) {
  // default: relay follows desiredOn
  if (p.desiredOn != p.relayOn) {
    p.relayOn = p.desiredOn;
    relayWrite(relayPin, p.relayOn);

    // reset timers
    if (p.relayOn) {
      p.onSinceMs = nowMs;
      p.flowBadSinceMs = 0;
    } else {
      p.flowWhileOffSinceMs = 0;
    }
  }
}

static void evaluateSafetyForPump(PumpState &p, uint32_t nowMs) {
  // --- Dry-run detection (pump ON but low flow) ---
  if (p.relayOn) {
    // grace period
    if (nowMs - p.onSinceMs >= DRYRUN_GRACE_MS) {
      if (p.flowLpm < MIN_FLOW_LPM) {
        if (p.flowBadSinceMs == 0) p.flowBadSinceMs = nowMs;
        if (nowMs - p.flowBadSinceMs >= DRYRUN_TIMEOUT_MS) {
          p.dryRun = true;
          p.desiredOn = false; // force OFF
        }
      } else {
        p.flowBadSinceMs = 0;
        p.dryRun = false;
      }
    }
  } else {
    p.flowBadSinceMs = 0;
  }

  // --- Relay stuck estimation (pump OFF but still has flow) ---
  if (!p.relayOn) {
    if (p.flowLpm > RELAY_STUCK_FLOW_LPM) {
      if (p.flowWhileOffSinceMs == 0) p.flowWhileOffSinceMs = nowMs;
      if (nowMs - p.flowWhileOffSinceMs >= RELAY_STUCK_TIMEOUT_MS) {
        p.relayStuck = true;
      }
    } else {
      p.flowWhileOffSinceMs = 0;
      p.relayStuck = false;
    }
  } else {
    p.flowWhileOffSinceMs = 0;

    p.relayStuck = false;
  }
}

static void applyFailSafe(uint32_t nowMs) {
  // Reset global error
  centralHasError = false;

  // Tank low -> force OFF cả 2 bơm
  bool tankLow = (tankFlagsBits & (TANK_FLAG_LOW_PCT | TANK_FLAG_LOW_SWITCH)) != 0;
  if (tankLow) {
    p1.desiredOn = false;
    p2.desiredOn = false;
    centralHasError = true;
  }

  // Per-pump safety
  evaluateSafetyForPump(p1, nowMs);
  evaluateSafetyForPump(p2, nowMs);

  if (p1.dryRun || p1.relayStuck || p2.dryRun || p2.relayStuck) {
    centralHasError = true;
  }
}

static int pumpStatusBits(const PumpState &p) {
  int bits = 0;
  if (p.relayOn)    bits |= PUMP_BIT_ON;
  if (p.dryRun)     bits |= PUMP_BIT_DRYRUN;
  if (p.relayStuck) bits |= PUMP_BIT_RELAY_STUCK;
  return bits;
}

// ======================================================
// 8) ESPNOW SEND REPORT
// ======================================================
static void sendMsg(int id, float temp, float hum, float soil) {
  struct_message msg;
  msg.id = id;
  msg.temp = temp;
  msg.hum  = hum;
  msg.soil = soil;
  esp_now_send(GATEWAY_MAC, (uint8_t *)&msg, sizeof(msg));
}

static void sendReport() {
  // Central summary: Gateway đang map thành keys: flow_rate, total_volume, pump_status
  // => gửi tổng flow và tổng volume.
  float totalFlow = p1.flowLpm + p2.flowLpm;
  float totalVol  = p1.totalL  + p2.totalL;

  int bitsPumps = 0;
  bitsPumps |= (pumpStatusBits(p1) & 0xFF) << 0;
  bitsPumps |= (pumpStatusBits(p2) & 0xFF) << 8;

  // Theo gateway: centralSumError = (soil == 2)
  float errorCode = centralHasError ? 2.0f : 0.0f;

  sendMsg(ID_CENTRAL_SUM, totalFlow, totalVol, errorCode);

  // Pump 1 / 2
  sendMsg(ID_PUMP1, p1.flowLpm, p1.totalL, (float)pumpStatusBits(p1));
  sendMsg(ID_PUMP2, p2.flowLpm, p2.totalL, (float)pumpStatusBits(p2));

  // Tank
  sendMsg(ID_TANK, tankLevelPct, tankLevelL, (float)tankFlagsBits);
}

// ======================================================
// 9) ESPNOW RECEIVE COMMAND
// ======================================================
static void onReceive(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (len != (int)sizeof(struct_command)) return;
  struct_command cmd;
  memcpy(&cmd, incomingData, sizeof(cmd));

  // cmd_id=1: set pump on/off
  if (cmd.cmd_id == 1) {
    bool on = (cmd.action == 1);

    if (cmd.pump_id == 1) p1.desiredOn = on;
    else if (cmd.pump_id == 2) p2.desiredOn = on;

    // Apply ngay để UI phản hồi nhanh
    uint32_t nowMs = millis();
    applyFailSafe(nowMs);
    applyRelay(p1, RELAY_1_PIN, nowMs);
    applyRelay(p2, RELAY_2_PIN, nowMs);
    sendReport();
  }
}

// ======================================================
// 10) SETUP / LOOP
// ======================================================
void setup() {
  Serial.begin(115200);
  delay(200);

  // Relay init
  pinMode(RELAY_1_PIN, OUTPUT);
  pinMode(RELAY_2_PIN, OUTPUT);
  // default OFF
  relayWrite(RELAY_1_PIN, false);
  relayWrite(RELAY_2_PIN, false);

  // Tank pins
  if (TANK_MODE == TANK_LEVEL_FLOAT_SWITCH) {
    pinMode(TANK_LOW_SWITCH_PIN, INPUT_PULLUP);
  } else {
    pinMode(TANK_LEVEL_ADC_PIN, INPUT);
    analogReadResolution(12);
    analogSetPinAttenuation(TANK_LEVEL_ADC_PIN, ADC_11db); // 0..3.3V (tuỳ board)
    //
    // pinMode(TANK_LOW_SWITCH_PIN, INPUT_PULLUP);
  }

  // Flow pins + interrupts
  pinMode(FLOW_1_PIN, INPUT); // Flow sensor thường cần pull-up; nếu pin không có pull-up nội (GPIO34/35) thì dùng điện trở kéo lên ngoài
  pinMode(FLOW_2_PIN, INPUT); // giống FLOW_1_PIN
  attachInterrupt(digitalPinToInterrupt(FLOW_1_PIN), pulseCounter1, FALLING);
  attachInterrupt(digitalPinToInterrupt(FLOW_2_PIN), pulseCounter2, FALLING);

  // Load persisted totals
  loadPersistent();

  // ESP-NOW init
  esp_base_mac_addr_set(CENTRAL_BASE_MAC);
  WiFi.mode(WIFI_STA);

  // Giữ channel cố định
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] init FAILED");
    while (true) delay(1000);
  }

  esp_now_register_recv_cb(onReceive);

  esp_now_peer_info_t peerInfo{};
  memcpy(peerInfo.peer_addr, GATEWAY_MAC, 6);
  peerInfo.channel = WIFI_CHANNEL;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("[ESP-NOW] add peer FAILED");
    while (true) delay(1000);
  }

  Serial.println("[Central] Ready (REAL HARDWARE)");
}

void loop() {
  static uint32_t lastTick = 0;
  static uint32_t lastReport = 0;
  static uint32_t lastSave = 0;

  const uint32_t now = millis();

  // 1) Tick: update sensors + safety
  if (now - lastTick >= TICK_MS) {
    uint32_t dt = now - lastTick;
    lastTick = now;

    updateTankLevel();
    updateFlowAndTotals(dt);
    applyFailSafe(now);

    // Apply relay after failsafe decision
    applyRelay(p1, RELAY_1_PIN, now);
    applyRelay(p2, RELAY_2_PIN, now);

    // Debug log ngắn gọn
    Serial.printf("[Central] tank=%.1f%% (%.0fL flags=%u) | P1(on=%d flow=%.2f tot=%.2f bits=%d) | P2(on=%d flow=%.2f tot=%.2f bits=%d) | err=%d\n",
                  tankLevelPct, tankLevelL, (unsigned)tankFlagsBits,
                  (int)p1.relayOn, p1.flowLpm, p1.totalL, pumpStatusBits(p1),
                  (int)p2.relayOn, p2.flowLpm, p2.totalL, pumpStatusBits(p2),
                  centralHasError ? 1 : 0);
  }

  // 2) Report to gateway
  if (now - lastReport >= REPORT_MS) {
    lastReport = now;
    sendReport();
  }

  // 3) Save totals to NVS (không lưu quá thường xuyên)
  if (now - lastSave >= SAVE_MS) {
    lastSave = now;
    savePersistent();
  }
}
