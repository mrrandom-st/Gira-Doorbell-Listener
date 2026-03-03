#include <M5Unified.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ===================== USER CONFIG =====================
#include "config.h"

#ifndef WIFI_SSID
#error "Bitte config.h anlegen (Kopie von config.example.h) und WIFI/MQTT Werte setzen."
#endif

// ===================== DISPLAY CONFIG =====================
#define BG_COLOR    TFT_BLACK
#define TEXT_COLOR  TFT_WHITE
#define ALERT_COLOR TFT_RED
#define OK_COLOR    TFT_GREEN
#define WARN_COLOR  TFT_YELLOW

// ===================== NETWORKING =====================
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

// ===================== STATE =====================
struct DoorbellState {
  bool online = false;
  uint32_t lastEventTime = 0;
  uint32_t lastStatusTime = 0;
  
  // Event data
  int32_t peak = 0;
  float b1 = 0, b2 = 0, b3 = 0;
  float ratio = 0;
  int tf = 0;
  
  // Status data
  int32_t peakGate = 0;
  float bandRatioMin = 0;
  float bandDomMin = 0;
  int tfNeed = 0;
  uint32_t cooldownMs = 0;
  int shift = 0;
  String ip = "--";
  
  // Alert state
  bool alertActive = false;
  uint32_t alertStartTime = 0;
  int alertCount = 0;
};

DoorbellState state;

// ===================== AUDIO =====================
#define ALERT_DURATION 3000  // 3 Sekunden Alert-Anzeige
#define BEEP_FREQ 1000       // 1 kHz Piepton
#define BEEP_DURATION 200    // 200ms Piepton

void playBeep() {
  M5.Speaker.tone(BEEP_FREQ, BEEP_DURATION);
}

void playAlert() {
  // Dreifacher Piepton
  M5.Speaker.tone(1000, 150);
  delay(200);
  M5.Speaker.tone(1200, 150);
  delay(200);
  M5.Speaker.tone(1400, 150);
}

// ===================== MQTT CALLBACKS =====================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // Parse JSON
  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, payload, length);
  
  if (error) {
    Serial.print("JSON parse error: ");
    Serial.println(error.c_str());
    return;
  }

  String topicStr = String(topic);
  
  // EVENT: Klingel wurde ausgelöst
  if (topicStr == TOPIC_EVENT) {
    if (doc["trigger"] == 1) {
      state.lastEventTime = millis();
      state.peak = doc["peak"] | 0;
      state.b1 = doc["b1"] | 0.0f;
      state.b2 = doc["b2"] | 0.0f;
      state.b3 = doc["b3"] | 0.0f;
      state.ratio = doc["ratio"] | 0.0f;
      state.tf = doc["tf"] | 0;
      
      // Alert aktivieren
      state.alertActive = true;
      state.alertStartTime = millis();
      state.alertCount++;
      
      // Sound abspielen
      playAlert();
      
      Serial.println("🔔 DOORBELL TRIGGERED!");
      Serial.printf("  Peak: %ld, Ratio: %.2f, TF: %d\n", 
                    (long)state.peak, state.ratio, state.tf);
    }
  }
  
  // STATUS: Regelmäßige Updates vom Sensor
  else if (topicStr == TOPIC_STATUS) {
    state.online = doc["online"] | false;
    state.lastStatusTime = millis();
    
    state.peakGate = doc["peakGate"] | 0;
    state.bandRatioMin = doc["bandRatioMin"] | 0.0f;
    state.bandDomMin = doc["bandDomMin"] | 0.0f;
    state.tfNeed = doc["tfNeed"] | 0;
    state.cooldownMs = doc["cool"] | 0;
    state.shift = doc["shift"] | 0;
    
    state.peak = doc["peak"] | 0;
    state.b1 = doc["b1"] | 0.0f;
    state.b2 = doc["b2"] | 0.0f;
    state.b3 = doc["b3"] | 0.0f;
    state.ratio = doc["ratio"] | 0.0f;
    state.tf = doc["tf"] | 0;
    
    state.ip = doc["ip"] | "--";
    
    Serial.println("📊 Status update received");
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
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✓ WiFi connected");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n✗ WiFi failed");
  }
}

void mqttConnect() {
  if (!WiFi.isConnected()) return;
  if (mqtt.connected()) return;
  
  Serial.print("Connecting to MQTT...");
  
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  mqtt.setBufferSize(512);
  
  String clientId = "m5stack-display-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  
  if (mqtt.connect(clientId.c_str(), MQTT_USER, MQTT_PASS)) {
    Serial.println(" ✓ connected");
    mqtt.subscribe(TOPIC_EVENT);
    mqtt.subscribe(TOPIC_STATUS);
    Serial.println("Subscribed to topics");
  } else {
    Serial.print(" ✗ failed, rc=");
    Serial.println(mqtt.state());
  }
}

// ===================== DISPLAY =====================
void drawHeader() {
  M5.Display.setTextSize(2);
  M5.Display.setTextColor(TFT_CYAN, BG_COLOR);
  M5.Display.setCursor(10, 10);
  M5.Display.print("DOORBELL MONITOR");
  
  // WiFi Status
  M5.Display.setTextSize(1);
  M5.Display.setCursor(10, 35);
  if (WiFi.isConnected()) {
    M5.Display.setTextColor(OK_COLOR, BG_COLOR);
    M5.Display.print("WiFi: OK");
  } else {
    M5.Display.setTextColor(ALERT_COLOR, BG_COLOR);
    M5.Display.print("WiFi: --");
  }
  
  // MQTT Status
  M5.Display.setCursor(120, 35);
  if (mqtt.connected()) {
    M5.Display.setTextColor(OK_COLOR, BG_COLOR);
    M5.Display.print("MQTT: OK");
  } else {
    M5.Display.setTextColor(ALERT_COLOR, BG_COLOR);
    M5.Display.print("MQTT: --");
  }
  
  // Sensor Online Status
  M5.Display.setCursor(230, 35);
  uint32_t timeSinceStatus = millis() - state.lastStatusTime;
  if (state.online && timeSinceStatus < 10000) {
    M5.Display.setTextColor(OK_COLOR, BG_COLOR);
    M5.Display.print("Sensor: OK");
  } else {
    M5.Display.setTextColor(WARN_COLOR, BG_COLOR);
    M5.Display.print("Sensor: --");
  }
  
  // Trennlinie
  M5.Display.drawLine(0, 50, M5.Display.width(), 50, TFT_DARKGREY);
}

void drawAlert() {
  if (!state.alertActive) return;
  
  // Alert-Timeout prüfen
  if (millis() - state.alertStartTime > ALERT_DURATION) {
    state.alertActive = false;
    return;
  }
  
  // Blinkendes Alert-Banner
  bool blink = ((millis() / 300) % 2) == 0;
  
  if (blink) {
    M5.Display.fillRect(0, 60, M5.Display.width(), 80, ALERT_COLOR);
    M5.Display.setTextSize(3);
    M5.Display.setTextColor(TFT_WHITE, ALERT_COLOR);
    M5.Display.setCursor(50, 80);
    M5.Display.print("DOORBELL!");
  } else {
    M5.Display.fillRect(0, 60, M5.Display.width(), 80, BG_COLOR);
  }
  
  // Event-Details
  M5.Display.setTextSize(2);
  M5.Display.setTextColor(TEXT_COLOR, BG_COLOR);
  M5.Display.setCursor(10, 150);
  M5.Display.printf("Peak: %ld", (long)state.peak);
  
  M5.Display.setCursor(10, 175);
  M5.Display.printf("Ratio: %.2f", state.ratio);
  
  M5.Display.setCursor(10, 200);
  M5.Display.printf("Frames: %d", state.tf);
}

void drawStatus() {
  if (state.alertActive) return;  // Nicht über Alert zeichnen
  
  M5.Display.setTextSize(1);
  M5.Display.setTextColor(TEXT_COLOR, BG_COLOR);
  
  int y = 70;
  
  // Sensor IP
  M5.Display.setCursor(10, y);
  M5.Display.print("Sensor IP: ");
  M5.Display.print(state.ip);
  y += 20;
  
  // Config
  M5.Display.setCursor(10, y);
  M5.Display.printf("Peak Gate: %ld", (long)state.peakGate);
  y += 15;
  
  M5.Display.setCursor(10, y);
  M5.Display.printf("Band Ratio Min: %.2f", state.bandRatioMin);
  y += 15;
  
  M5.Display.setCursor(10, y);
  M5.Display.printf("Band Dom Min: %.2f", state.bandDomMin);
  y += 15;
  
  M5.Display.setCursor(10, y);
  M5.Display.printf("Tonal Frames: %d", state.tfNeed);
  y += 15;
  
  M5.Display.setCursor(10, y);
  M5.Display.printf("Cooldown: %u ms", (unsigned)state.cooldownMs);
  y += 15;
  
  M5.Display.setCursor(10, y);
  M5.Display.printf("Shift: %d", state.shift);
  y += 25;
  
  // Current values
  M5.Display.setTextColor(TFT_YELLOW, BG_COLOR);
  M5.Display.setCursor(10, y);
  M5.Display.print("CURRENT VALUES:");
  y += 20;
  
  M5.Display.setTextColor(TEXT_COLOR, BG_COLOR);
  M5.Display.setCursor(10, y);
  M5.Display.printf("Peak: %ld", (long)state.peak);
  y += 15;
  
  M5.Display.setCursor(10, y);
  M5.Display.printf("B1: %.0f  B2: %.0f  B3: %.0f", state.b1, state.b2, state.b3);
  y += 15;
  
  M5.Display.setCursor(10, y);
  M5.Display.printf("Ratio: %.2f  TF: %d", state.ratio, state.tf);
  y += 25;
  
  // Alert Counter
  M5.Display.setTextColor(TFT_CYAN, BG_COLOR);
  M5.Display.setCursor(10, y);
  M5.Display.printf("Total Alerts: %d", state.alertCount);
  
  // Last Event
  if (state.lastEventTime > 0) {
    uint32_t elapsed = (millis() - state.lastEventTime) / 1000;
    M5.Display.setCursor(200, y);
    M5.Display.printf("Last: %us ago", (unsigned)elapsed);
  }
}

void drawFooter() {
  int y = M5.Display.height() - 20;
  M5.Display.drawLine(0, y - 5, M5.Display.width(), y - 5, TFT_DARKGREY);
  
  M5.Display.setTextSize(1);
  M5.Display.setTextColor(TFT_DARKGREY, BG_COLOR);
  M5.Display.setCursor(10, y);
  M5.Display.print("BtnA: Test  BtnB: Clear  BtnC: Reconnect");
}

void updateDisplay() {
  M5.Display.fillScreen(BG_COLOR);
  
  drawHeader();
  drawAlert();
  drawStatus();
  drawFooter();
}

// ===================== BUTTON HANDLERS =====================
void handleButtons() {
  M5.update();
  
  // Button A: Test Alert
  if (M5.BtnA.wasPressed()) {
    Serial.println("Button A: Test Alert");
    state.alertActive = true;
    state.alertStartTime = millis();
    state.alertCount++;
    state.peak = 9999;
    state.ratio = 0.99f;
    state.tf = 10;
    playAlert();
  }
  
  // Button B: Clear Alert Counter
  if (M5.BtnB.wasPressed()) {
    Serial.println("Button B: Clear Counter");
    state.alertCount = 0;
    playBeep();
  }
  
  // Button C: Reconnect MQTT
  if (M5.BtnC.wasPressed()) {
    Serial.println("Button C: Reconnect");
    mqtt.disconnect();
    delay(100);
    mqttConnect();
    playBeep();
  }
}

// ===================== SETUP =====================
void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  
  M5.Display.setRotation(1);  // Landscape
  M5.Display.setTextDatum(TL_DATUM);
  M5.Display.fillScreen(BG_COLOR);
  
  M5.Display.setTextSize(2);
  M5.Display.setTextColor(TFT_WHITE);
  M5.Display.setCursor(50, 100);
  M5.Display.print("Doorbell Monitor");
  M5.Display.setCursor(80, 130);
  M5.Display.print("Starting...");
  
  Serial.begin(115200);
  delay(500);
  
  Serial.println("\n=== M5Stack Core S3 SE - Doorbell Monitor ===");
  
  // WiFi verbinden
  wifiConnect();
  
  // MQTT verbinden
  mqttConnect();
  
  delay(1000);
  updateDisplay();
}

// ===================== LOOP =====================
void loop() {
  // Buttons prüfen
  handleButtons();
  
  // WiFi Reconnect (alle 5 Sekunden prüfen)
  static uint32_t lastWifiCheck = 0;
  if (millis() - lastWifiCheck > 5000) {
    lastWifiCheck = millis();
    if (!WiFi.isConnected()) {
      Serial.println("WiFi lost, reconnecting...");
      wifiConnect();
    }
  }
  
  // MQTT Reconnect (alle 5 Sekunden prüfen)
  static uint32_t lastMqttCheck = 0;
  if (millis() - lastMqttCheck > 5000) {
    lastMqttCheck = millis();
    if (WiFi.isConnected() && !mqtt.connected()) {
      Serial.println("MQTT lost, reconnecting...");
      mqttConnect();
    }
  }
  
  // MQTT Loop
  if (mqtt.connected()) {
    mqtt.loop();
  }
  
  // Display Update (alle 200ms)
  static uint32_t lastDisplayUpdate = 0;
  if (millis() - lastDisplayUpdate > 200) {
    lastDisplayUpdate = millis();
    updateDisplay();
  }
  
  // Sensor Timeout Check (kein Status seit 30 Sekunden)
  if (state.online && (millis() - state.lastStatusTime > 30000)) {
    state.online = false;
    Serial.println("⚠ Sensor timeout - no status for 30s");
  }
  
  delay(50);
}
