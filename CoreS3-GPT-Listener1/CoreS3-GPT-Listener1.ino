/*
  M5Stack Core S3 SE - Doorbell Monitor (improved)

  Verbesserungen ggü. deiner Version:
  - Robusteres JSON-Parsing (Whitespace-tolerant, String korrekt)
  - Keine blockierenden delay()-Sounds mehr (non-blocking Alert-Sound State-Machine)
  - Weniger Display-Flackern: kein Fullscreen-Redraw; nur Teilbereiche werden aktualisiert
  - Heap-schonender MQTT-Payload-String (reserve)
  - Sauberere Sensor-Online-Logik: "reported online" + "alive by timeout"
  - Zusätzliche Debug-Werte: Status-Alter (sec), Triggers/Minute
*/

#include <M5Unified.h>
#include <WiFi.h>
#include <PubSubClient.h>

// ===================== USER CONFIG =====================
static const char* WIFI_SSID = "";
static const char* WIFI_PASS = "";

static const char* MQTT_HOST = "";
static const uint16_t MQTT_PORT = 1885;
static const char* MQTT_USER = "";
static const char* MQTT_PASS = "";

static const char* TOPIC_EVENT  = "home/doorbell/event";
static const char* TOPIC_STATUS = "home/doorbell/status";

// ===================== DISPLAY CONFIG =====================
#define BG_COLOR    TFT_BLACK
#define TEXT_COLOR  TFT_WHITE
#define ALERT_COLOR TFT_RED
#define OK_COLOR    TFT_GREEN
#define WARN_COLOR  TFT_YELLOW

// Layout (Rotation 1: 320x240 bei CoreS3 typischerweise)
static const int W = 320;
static const int H = 240;

// Bereiche
static const int HEADER_Y0 = 0;
static const int HEADER_H  = 52;

static const int BODY_Y0   = HEADER_Y0 + HEADER_H;   // 52
static const int BODY_H    = 168;                    // bis 220

static const int FOOT_Y0   = 220;
static const int FOOT_H    = 20;

// Alarm-Bereich (oben im Body)
static const int ALARM_Y0  = BODY_Y0;
static const int ALARM_H   = 80;

// Status-Bereich (unter Alarm)
static const int STAT_Y0   = BODY_Y0 + ALARM_H;
static const int STAT_H    = BODY_H - ALARM_H;

// ===================== NETWORKING =====================
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

// ===================== HELPERS =====================
static inline bool isWS(char c) { return c == ' ' || c == '\t' || c == '\r' || c == '\n'; }

// Findet den Startindex des Werts hinter "key":  (Whitespace tolerant)
// Rückgabe: Index auf erstes Zeichen des Werts (z.B. '1', '-', '"', '{' ...), oder -1.
int findJsonValueStart(const String& json, const String& key) {
  const String needle = "\"" + key + "\"";
  int k = json.indexOf(needle);
  if (k < 0) return -1;

  int colon = json.indexOf(':', k + needle.length());
  if (colon < 0) return -1;

  int i = colon + 1;
  while (i < (int)json.length() && isWS(json[i])) i++;
  if (i >= (int)json.length()) return -1;
  return i;
}

float parseJsonFloat(const String& json, const String& key, float def = 0.0f) {
  int i = findJsonValueStart(json, key);
  if (i < 0) return def;

  int end = i;
  while (end < (int)json.length()) {
    char c = json[end];
    if (c == ',' || c == '}' || c == ']') break;
    end++;
  }

  String s = json.substring(i, end);
  s.trim();
  if (s.length() == 0) return def;
  return s.toFloat();
}

int parseJsonInt(const String& json, const String& key, int def = 0) {
  int i = findJsonValueStart(json, key);
  if (i < 0) return def;

  int end = i;
  while (end < (int)json.length()) {
    char c = json[end];
    if (c == ',' || c == '}' || c == ']') break;
    end++;
  }

  String s = json.substring(i, end);
  s.trim();
  if (s.length() == 0) return def;
  return s.toInt();
}

String parseJsonString(const String& json, const String& key, const String& def = "--") {
  int i = findJsonValueStart(json, key);
  if (i < 0) return def;

  // Erwartet "..."
  if (json[i] != '"') return def;
  i++; // hinter das erste "

  int end = i;
  while (end < (int)json.length()) {
    char c = json[end];
    if (c == '"' && json[end - 1] != '\\') break;
    end++;
  }
  if (end >= (int)json.length()) return def;
  String s = json.substring(i, end);
  s.replace("\\\"", "\"");
  return s;
}

// ===================== STATE =====================
struct DoorbellState {
  // Connectivity
  bool wifiOk = false;
  bool mqttOk = false;

  // Sensor status
  bool sensorOnlineReported = false;   // aus JSON "online"
  bool sensorAliveByTimeout = false;   // lokal: status nicht zu alt
  uint32_t lastStatusMs = 0;

  // Last event
  uint32_t lastEventMs = 0;

  // Event/status values (latest)
  int32_t peak = 0;
  float b1 = 0, b2 = 0, b3 = 0;
  float ratio = 0;
  int tf = 0;

  // Status config values
  int32_t peakGate = 0;
  float bandRatioMin = 0;
  float bandDomMin = 0;
  int tfNeed = 0;
  uint32_t cooldownMs = 0;
  int shift = 0;
  String ip = "--";

  // Alert visual
  bool alertActive = false;
  uint32_t alertStartMs = 0;
  int alertCount = 0;

  // Triggers per minute (simple ring buffer timestamps)
  static const int MAX_TRIG = 30;
  uint32_t trigTimes[MAX_TRIG] = {0};
  int trigHead = 0;

  // Dirty flags for display
  bool dirtyHeader = true;
  bool dirtyAlarm  = true;
  bool dirtyStatus = true;
  bool dirtyFooter = true;
};

DoorbellState state;

// ===================== AUDIO (NON-BLOCKING) =====================
#define ALERT_DURATION_MS 3000

// Alert pattern steps (freq, toneDuration, gapAfter)
struct ToneStep { uint16_t f; uint16_t d; uint16_t g; };
static const ToneStep ALERT_STEPS[] = {
  {1000, 150, 120},
  {1200, 150, 120},
  {1400, 150, 0},
};
static const int ALERT_STEP_COUNT = sizeof(ALERT_STEPS) / sizeof(ALERT_STEPS[0]);

struct TonePlayer {
  bool active = false;
  int step = 0;
  uint32_t nextMs = 0;
};

TonePlayer tonePlayer;

void startAlertTone() {
  tonePlayer.active = true;
  tonePlayer.step = 0;
  tonePlayer.nextMs = 0; // start immediately
}

void startBeep(uint16_t freq = 1000, uint16_t dur = 120) {
  // kurzer Einzelton (auch non-blocking)
  M5.Speaker.tone(freq, dur);
}

void updateTonePlayer() {
  if (!tonePlayer.active) return;
  uint32_t now = millis();
  if (tonePlayer.nextMs != 0 && (int32_t)(now - tonePlayer.nextMs) < 0) return;

  if (tonePlayer.step >= ALERT_STEP_COUNT) {
    tonePlayer.active = false;
    return;
  }

  const ToneStep& s = ALERT_STEPS[tonePlayer.step];
  M5.Speaker.tone(s.f, s.d);
  tonePlayer.nextMs = now + s.d + s.g;
  tonePlayer.step++;
}

// ===================== MQTT CALLBACK =====================
void addTriggerTimestamp(uint32_t nowMs) {
  state.trigTimes[state.trigHead] = nowMs;
  state.trigHead = (state.trigHead + 1) % DoorbellState::MAX_TRIG;
}

int triggersLastMinute(uint32_t nowMs) {
  int c = 0;
  for (int i = 0; i < DoorbellState::MAX_TRIG; i++) {
    uint32_t t = state.trigTimes[i];
    if (t != 0 && (nowMs - t) <= 60000UL) c++;
  }
  return c;
}

void onDoorbellTriggeredFromJson(const String& json) {
  state.lastEventMs = millis();
  state.peak  = parseJsonInt(json, "peak", state.peak);
  state.b1    = parseJsonFloat(json, "b1", state.b1);
  state.b2    = parseJsonFloat(json, "b2", state.b2);
  state.b3    = parseJsonFloat(json, "b3", state.b3);
  state.ratio = parseJsonFloat(json, "ratio", state.ratio);
  state.tf    = parseJsonInt(json, "tf", state.tf);

  state.alertActive  = true;
  state.alertStartMs = millis();
  state.alertCount++;
  addTriggerTimestamp(state.alertStartMs);

  startAlertTone();

  state.dirtyAlarm  = true;
  state.dirtyStatus = true;
}

void onStatusFromJson(const String& json) {
  state.sensorOnlineReported = (parseJsonInt(json, "online", 0) == 1);
  state.lastStatusMs = millis();

  state.peakGate      = parseJsonInt(json, "peakGate", state.peakGate);
  state.bandRatioMin  = parseJsonFloat(json, "bandRatioMin", state.bandRatioMin);
  state.bandDomMin    = parseJsonFloat(json, "bandDomMin", state.bandDomMin);
  state.tfNeed        = parseJsonInt(json, "tfNeed", state.tfNeed);
  state.cooldownMs    = (uint32_t)parseJsonInt(json, "cool", (int)state.cooldownMs);
  state.shift         = parseJsonInt(json, "shift", state.shift);
  state.ip            = parseJsonString(json, "ip", state.ip);

  // Optional: live values may also be included in status
  state.peak  = parseJsonInt(json, "peak", state.peak);
  state.b1    = parseJsonFloat(json, "b1", state.b1);
  state.b2    = parseJsonFloat(json, "b2", state.b2);
  state.b3    = parseJsonFloat(json, "b3", state.b3);
  state.ratio = parseJsonFloat(json, "ratio", state.ratio);
  state.tf    = parseJsonInt(json, "tf", state.tf);

  state.dirtyHeader = true;
  state.dirtyStatus = true;
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String json;
  json.reserve(length + 1);
  for (unsigned int i = 0; i < length; i++) json += (char)payload[i];

  const String t = String(topic);

  if (t == TOPIC_EVENT) {
    if (parseJsonInt(json, "trigger", 0) == 1) {
      Serial.println("🔔 EVENT: trigger=1");
      onDoorbellTriggeredFromJson(json);
    }
  } else if (t == TOPIC_STATUS) {
    Serial.println("📊 STATUS update");
    onStatusFromJson(json);
  }
}

// ===================== WIFI & MQTT =====================
void wifiConnect() {
  if (WiFi.status() == WL_CONNECTED) return;

  Serial.print("Connecting to WiFi");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(300);
    Serial.print(".");
    attempts++;
  }

  state.wifiOk = (WiFi.status() == WL_CONNECTED);
  if (state.wifiOk) {
    Serial.println("\n✓ WiFi connected");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n✗ WiFi failed");
  }
  state.dirtyHeader = true;
}

void mqttConnect() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (mqtt.connected()) return;

  Serial.print("Connecting to MQTT... ");

  // setServer/callback nur einmal wäre auch ok, aber so ist es robust bei Re-init
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  mqtt.setBufferSize(1024);
  mqtt.setKeepAlive(15);

  String clientId = "m5stack-display-" + String((uint32_t)ESP.getEfuseMac(), HEX);

  if (mqtt.connect(clientId.c_str(), MQTT_USER, MQTT_PASS)) {
    Serial.println("✓ connected");
    mqtt.subscribe(TOPIC_EVENT);
    mqtt.subscribe(TOPIC_STATUS);
    state.mqttOk = true;
  } else {
    Serial.print("✗ failed rc=");
    Serial.println(mqtt.state());
    state.mqttOk = false;
  }
  state.dirtyHeader = true;
}

// ===================== DISPLAY DRAW =====================
void drawHeaderStatic() {
  M5.Display.fillRect(0, HEADER_Y0, W, HEADER_H, BG_COLOR);
  M5.Display.setTextSize(2);
  M5.Display.setTextColor(TFT_CYAN, BG_COLOR);
  M5.Display.setCursor(10, 10);
  M5.Display.print("DOORBELL MONITOR");
  M5.Display.drawLine(0, HEADER_H - 2, W, HEADER_H - 2, TFT_DARKGREY);
}

void drawFooterStatic() {
  M5.Display.fillRect(0, FOOT_Y0, W, FOOT_H, BG_COLOR);
  M5.Display.drawLine(0, FOOT_Y0 - 2, W, FOOT_Y0 - 2, TFT_DARKGREY);
  M5.Display.setTextSize(1);
  M5.Display.setTextColor(TFT_DARKGREY, BG_COLOR);
  M5.Display.setCursor(10, FOOT_Y0 + 4);
  M5.Display.print("BtnA: Test  BtnB: Clear  BtnC: Reconnect");
}

void drawHeaderDynamic() {
  // Clear area under title (second line area)
  M5.Display.fillRect(0, 32, W, 18, BG_COLOR);
  M5.Display.setTextSize(1);

  // WiFi
  M5.Display.setCursor(10, 35);
  if (WiFi.status() == WL_CONNECTED) {
    M5.Display.setTextColor(OK_COLOR, BG_COLOR);
    M5.Display.print("WiFi: OK");
  } else {
    M5.Display.setTextColor(ALERT_COLOR, BG_COLOR);
    M5.Display.print("WiFi: --");
  }

  // MQTT
  M5.Display.setCursor(120, 35);
  if (mqtt.connected()) {
    M5.Display.setTextColor(OK_COLOR, BG_COLOR);
    M5.Display.print("MQTT: OK");
  } else {
    M5.Display.setTextColor(ALERT_COLOR, BG_COLOR);
    M5.Display.print("MQTT: --");
  }

  // Sensor
  uint32_t now = millis();
  uint32_t ageMs = (state.lastStatusMs == 0) ? 999999UL : (now - state.lastStatusMs);
  state.sensorAliveByTimeout = (ageMs < 10000UL);

  M5.Display.setCursor(230, 35);
  if (state.sensorOnlineReported && state.sensorAliveByTimeout) {
    M5.Display.setTextColor(OK_COLOR, BG_COLOR);
    M5.Display.print("Sensor: OK");
  } else {
    M5.Display.setTextColor(WARN_COLOR, BG_COLOR);
    M5.Display.print("Sensor: --");
  }
}

void clearAlarmArea() {
  M5.Display.fillRect(0, ALARM_Y0, W, ALARM_H, BG_COLOR);
}

void drawAlarmArea() {
  uint32_t now = millis();

  if (state.alertActive && (now - state.alertStartMs > ALERT_DURATION_MS)) {
    state.alertActive = false;
    state.dirtyAlarm = true;
    state.dirtyStatus = true;
  }

  // blink
  bool blink = ((now / 300) % 2) == 0;

  // Clear first
  clearAlarmArea();

  if (state.alertActive && blink) {
    M5.Display.fillRect(0, ALARM_Y0, W, ALARM_H, ALERT_COLOR);
    M5.Display.setTextSize(3);
    M5.Display.setTextColor(TFT_WHITE, ALERT_COLOR);
    M5.Display.setCursor(55, ALARM_Y0 + 18);
    M5.Display.print("DOORBELL!");
  } else {
    // Show last event summary even when no blink
    M5.Display.setTextSize(2);
    M5.Display.setTextColor(TEXT_COLOR, BG_COLOR);
    M5.Display.setCursor(10, ALARM_Y0 + 10);
    M5.Display.printf("Peak: %ld", (long)state.peak);
    M5.Display.setCursor(10, ALARM_Y0 + 35);
    M5.Display.printf("Ratio: %.2f", state.ratio);
    M5.Display.setCursor(10, ALARM_Y0 + 58);
    M5.Display.printf("Frames: %d", state.tf);
  }
}

void clearStatusArea() {
  M5.Display.fillRect(0, STAT_Y0, W, STAT_H, BG_COLOR);
}

void drawStatusArea() {
  clearStatusArea();

  M5.Display.setTextSize(1);
  M5.Display.setTextColor(TEXT_COLOR, BG_COLOR);

  int y = STAT_Y0 + 5;

  uint32_t now = millis();
  uint32_t statusAgeSec = (state.lastStatusMs == 0) ? 9999U : (now - state.lastStatusMs) / 1000U;
  int tpm = triggersLastMinute(now);

  // Sensor IP + Status age
  M5.Display.setCursor(10, y);
  M5.Display.printf("Sensor IP: %s", state.ip.c_str());
  y += 14;

  M5.Display.setCursor(10, y);
  M5.Display.printf("Status age: %us   Trig/min: %d", (unsigned)statusAgeSec, tpm);
  y += 16;

  // Config values
  M5.Display.setCursor(10, y);
  M5.Display.printf("PeakGate: %ld   Cool: %ums", (long)state.peakGate, (unsigned)state.cooldownMs);
  y += 14;

  M5.Display.setCursor(10, y);
  M5.Display.printf("RatioMin: %.2f  DomMin: %.2f", state.bandRatioMin, state.bandDomMin);
  y += 14;

  M5.Display.setCursor(10, y);
  M5.Display.printf("TF need: %d   Shift: %d", state.tfNeed, state.shift);
  y += 18;

  // Live values block
  M5.Display.setTextColor(TFT_YELLOW, BG_COLOR);
  M5.Display.setCursor(10, y);
  M5.Display.print("CURRENT VALUES:");
  y += 14;

  M5.Display.setTextColor(TEXT_COLOR, BG_COLOR);
  M5.Display.setCursor(10, y);
  M5.Display.printf("Peak: %ld   Ratio: %.2f   TF: %d", (long)state.peak, state.ratio, state.tf);
  y += 14;

  M5.Display.setCursor(10, y);
  M5.Display.printf("B1: %.0f  B2: %.0f  B3: %.0f", state.b1, state.b2, state.b3);
  y += 18;

  // Counters
  M5.Display.setTextColor(TFT_CYAN, BG_COLOR);
  M5.Display.setCursor(10, y);
  M5.Display.printf("Total Alerts: %d", state.alertCount);

  if (state.lastEventMs > 0) {
    uint32_t elapsed = (now - state.lastEventMs) / 1000U;
    M5.Display.setCursor(200, y);
    M5.Display.printf("Last: %us ago", (unsigned)elapsed);
  }
}

// ===================== BUTTONS =====================
void handleButtons() {
  M5.update();

  if (M5.BtnA.wasPressed()) {
    Serial.println("Button A: Test Alert");
    state.peak = 9999;
    state.ratio = 0.99f;
    state.tf = 10;
    state.b1 = 1111;
    state.b2 = 2222;
    state.b3 = 3333;

    state.alertActive = true;
    state.alertStartMs = millis();
    state.alertCount++;
    addTriggerTimestamp(state.alertStartMs);

    startAlertTone();

    state.dirtyAlarm = true;
    state.dirtyStatus = true;
  }

  if (M5.BtnB.wasPressed()) {
    Serial.println("Button B: Clear Counter");
    state.alertCount = 0;
    // Clear trigger history
    for (int i = 0; i < DoorbellState::MAX_TRIG; i++) state.trigTimes[i] = 0;
    state.trigHead = 0;

    startBeep(900, 120);
    state.dirtyStatus = true;
  }

  if (M5.BtnC.wasPressed()) {
    Serial.println("Button C: Reconnect");
    mqtt.disconnect();
    delay(50);
    mqttConnect();
    startBeep(1100, 120);
    state.dirtyHeader = true;
  }
}

// ===================== SETUP/LOOP =====================
void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);

  M5.Display.setRotation(1);
  M5.Display.setTextDatum(TL_DATUM);
  M5.Display.fillScreen(BG_COLOR);

  Serial.begin(115200);
  delay(200);

  Serial.println("\n=== M5Stack Core S3 SE - Doorbell Monitor (improved) ===");

  drawHeaderStatic();
  drawFooterStatic();
  clearAlarmArea();
  clearStatusArea();

  wifiConnect();
  mqttConnect();

  state.dirtyHeader = true;
  state.dirtyAlarm  = true;
  state.dirtyStatus = true;
}

void loop() {
  handleButtons();

  // Non-blocking audio
  updateTonePlayer();

  // Periodic WiFi check
  static uint32_t lastWifiCheck = 0;
  if (millis() - lastWifiCheck > 5000) {
    lastWifiCheck = millis();
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi lost, reconnecting...");
      wifiConnect();
    }
  }

  // Periodic MQTT check
  static uint32_t lastMqttCheck = 0;
  if (millis() - lastMqttCheck > 3000) {
    lastMqttCheck = millis();
    if (WiFi.status() == WL_CONNECTED && !mqtt.connected()) {
      Serial.println("MQTT lost, reconnecting...");
      mqttConnect();
    }
  }

  if (mqtt.connected()) mqtt.loop();

  // Sensor timeout influences header/status
  static uint32_t lastSensorTimeoutCheck = 0;
  if (millis() - lastSensorTimeoutCheck > 1000) {
    lastSensorTimeoutCheck = millis();
    uint32_t now = millis();
    bool alive = (state.lastStatusMs != 0 && (now - state.lastStatusMs) < 30000UL);
    // Wenn sich das ändert: redraw
    if (alive != state.sensorAliveByTimeout) {
      state.sensorAliveByTimeout = alive;
      state.dirtyHeader = true;
      state.dirtyStatus = true;
      if (!alive) Serial.println("⚠ Sensor timeout - no status for 30s");
    }
  }

  // Display updates (dirty only) + minimal periodic refresh for blink/status age
  static uint32_t lastUiTick = 0;
  bool uiTick = false;
  if (millis() - lastUiTick > 250) { // blink/age refresh
    lastUiTick = millis();
    uiTick = true;
  }

  if (state.dirtyHeader || uiTick) {
    drawHeaderDynamic();
    state.dirtyHeader = false;
  }

  // Alarm area: needs periodic refresh for blinking
  if (state.dirtyAlarm || uiTick) {
    drawAlarmArea();
    state.dirtyAlarm = false;
  }

  // Status area: update when dirty or once per second-ish (for status age / trig/min)
  static uint32_t lastStatusUiRefresh = 0;
  if (state.dirtyStatus || (millis() - lastStatusUiRefresh > 1000)) {
    lastStatusUiRefresh = millis();
    drawStatusArea();
    state.dirtyStatus = false;
  }

  delay(10);
}
