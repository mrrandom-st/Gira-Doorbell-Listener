#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <PubSubClient.h>

#include "driver/i2s.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ======================================================
//  ESP32 Doorbell - STUFE 1 TRIGGER TESTER (robust)
//  Trigger = Laut (peakGate) + tonal (dom/pk) + "HOLD" über mehrere Frames
//  -> schützt gegen Klatschen/Knacksen (Impulse)
// ======================================================

// ===================== USER CONFIG =====================
static const char* WIFI_SSID = "R2DLan";
static const char* WIFI_PASS = "#Stregner18!";

static const char* MQTT_HOST = "192.168.2.142";
static const uint16_t MQTT_PORT = 1885;
static const char* MQTT_USER = "randommqtt";
static const char* MQTT_PASS = "cH1Ll0U7";

static const char* TOPIC_EVENT  = "home/doorbell/event";
static const char* TOPIC_STATUS = "home/doorbell/status";

// ===================== OLED =====================
#define OLED_W 128
#define OLED_H 64
#define OLED_RESET -1
Adafruit_SSD1306 display(OLED_W, OLED_H, &Wire, OLED_RESET);

#define I2C_SDA 21
#define I2C_SCL 22

// ===================== I2S / INMP441 =====================
#define I2S_WS   25
#define I2S_SCK  32
#define I2S_SD   33
#define I2S_PORT I2S_NUM_0

// ===================== DSP / Trigger Test =====================
static const float FS = 16000.0f;
static const int   N  = 512;                 // ~32ms pro Frame

static int SHIFT = 16;

// Lautstärke: "niedrig empfindlich" -> hoher Gate
static int32_t peakGate = 3000;              // wie von dir gewünscht (ggf. 4000/5000)
static uint32_t cooldownMs = 1500;           // Sperre nach Trigger

// Klingel-Band (Goertzel Sensoren, breit genug)
static const int FCNT = 12;
static const float F[FCNT] = { 350, 450, 550, 650, 750, 850, 950, 1050, 1150, 1250, 1400, 1550 };

// Tonalität: dom = best/second, pk = best/sumBand
static float domThr  = 1.45f;                // tolerant genug für dom~1.5
static float pkThr   = 0.30f;                // passt zu deinen Klingelwerten (>=0.33)

// Anti-Klatschen: tonal muss X Frames am Stück anliegen
static int tonalNeedFrames = 5;              // 5 Frames ~160ms (7 => ~224ms)

// ===================== Networking =====================
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);
WebServer server(80);

// ===================== Goertzel =====================
struct GoertzelBin { float coeff, cosw, sinw; };
static GoertzelBin bins[FCNT];

static GoertzelBin makeBin(float f0) {
  int k = (int)(0.5f + (N * f0) / FS);
  float w = 2.0f * PI * (float)k / (float)N;
  GoertzelBin b{};
  b.cosw = cosf(w);
  b.sinw = sinf(w);
  b.coeff = 2.0f * b.cosw;
  return b;
}

static float goertzel_mag2(const int32_t* x, const GoertzelBin& b) {
  float s_prev = 0.0f, s_prev2 = 0.0f;
  for (int i = 0; i < N; i++) {
    float s = (float)x[i] + b.coeff * s_prev - s_prev2;
    s_prev2 = s_prev;
    s_prev  = s;
  }
  float real = s_prev - s_prev2 * b.cosw;
  float imag = s_prev2 * b.sinw;
  return real * real + imag * imag;
}

// ===================== Buffers =====================
static int32_t rawBuf[N];
static int32_t xBuf[N];

// ===================== Helpers =====================
static bool wifiOk() { return WiFi.status() == WL_CONNECTED; }
static bool mqttOk() { return mqtt.connected(); }

static void wifiEnsure() {
  if (wifiOk()) return;
  static bool started = false;
  if (!started) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    started = true;
  }
}

static void mqttEnsure() {
  if (!wifiOk()) return;
  if (mqttOk()) return;

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  String clientId = "doorbell-trig-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  mqtt.connect(clientId.c_str(), MQTT_USER, MQTT_PASS);
}

// ===================== I2S =====================
static void i2sSetup() {
  i2s_config_t cfg = {};
  cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
  cfg.sample_rate = (int)FS;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;
  cfg.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
  cfg.communication_format = (i2s_comm_format_t)I2S_COMM_FORMAT_STAND_I2S;
  cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  cfg.dma_buf_count = 8;
  cfg.dma_buf_len   = N;

  i2s_pin_config_t pin = {};
  pin.bck_io_num = I2S_SCK;
  pin.ws_io_num = I2S_WS;
  pin.data_out_num = -1;
  pin.data_in_num = I2S_SD;

  i2s_driver_install(I2S_PORT, &cfg, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin);
  i2s_set_clk(I2S_PORT, (uint32_t)FS, I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_MONO);
  i2s_zero_dma_buffer(I2S_PORT);
  i2s_start(I2S_PORT);
}

// ===================== MQTT publish =====================
static void publishTrig(int32_t peak, float bestF, float dom, float pk, int tf) {
  if (!mqttOk()) return;
  char buf[256];
  snprintf(buf, sizeof(buf),
           "{\"trigger\":1,\"peak\":%ld,\"bestF\":%.0f,\"dom\":%.2f,\"pk\":%.2f,\"tf\":%d}",
           (long)peak, bestF, dom, pk, tf);
  mqtt.publish(TOPIC_EVENT, buf, false);
}

static void publishStatus(int32_t peak, float bestF, float dom, float pk, int tf) {
  if (!mqttOk()) return;

  String ip = wifiOk() ? WiFi.localIP().toString() : "--";
  char buf[512];
  snprintf(buf, sizeof(buf),
           "{\"online\":1,"
           "\"peakGate\":%ld,\"domThr\":%.2f,\"pkThr\":%.2f,\"tfNeed\":%d,\"cool\":%u,\"shift\":%d,"
           "\"peak\":%ld,\"bestF\":%.0f,\"dom\":%.2f,\"pk\":%.2f,\"tf\":%d,"
           "\"ip\":\"%s\"}",
           (long)peakGate, domThr, pkThr, tonalNeedFrames, (unsigned)cooldownMs, SHIFT,
           (long)peak, bestF, dom, pk, tf,
           ip.c_str());
  mqtt.publish(TOPIC_STATUS, buf, true);
}

// ===================== Web (Tuning ohne Flash) =====================
// /set?pg=3500&dom=1.6&pk=0.35&tf=6&cd=1500&shift=16
static void handleSet() {
  if (server.hasArg("pg"))    peakGate = server.arg("pg").toInt();
  if (server.hasArg("dom"))   domThr = server.arg("dom").toFloat();
  if (server.hasArg("pk"))    pkThr = server.arg("pk").toFloat();
  
  if (server.hasArg("tf")) {
    int val = server.arg("tf").toInt();
    tonalNeedFrames = (val > 2) ? val : 2;
  }
  
  if (server.hasArg("cd"))    cooldownMs = (uint32_t)server.arg("cd").toInt();
  
  if (server.hasArg("shift")) {
    int val = server.arg("shift").toInt();
    SHIFT = (val < 10) ? 10 : (val > 20) ? 20 : val;
  }

  String s = "OK ";
  s += "pg=" + String((long)peakGate);
  s += " dom=" + String(domThr, 2);
  s += " pk=" + String(pkThr, 2);
  s += " tf=" + String(tonalNeedFrames);
  s += " cd=" + String((unsigned)cooldownMs);
  s += " shift=" + String(SHIFT);
  server.send(200, "text/plain", s);
}

static void handleStatus() {
  String s = "{";
  s += "\"wifi\":" + String(wifiOk()?1:0) + ",";
  s += "\"mqtt\":" + String(mqttOk()?1:0) + ",";
  s += "\"peakGate\":" + String((long)peakGate) + ",";
  s += "\"domThr\":" + String(domThr, 2) + ",";
  s += "\"pkThr\":" + String(pkThr, 2) + ",";
  s += "\"tfNeed\":" + String(tonalNeedFrames) + ",";
  s += "\"cooldownMs\":" + String((unsigned)cooldownMs) + ",";
  s += "\"shift\":" + String(SHIFT);
  s += "}";
  server.send(200, "application/json", s);
}

// ===================== OLED =====================
static void oledDraw(int32_t peak, bool loud,
                     float bestF, float dom, float pk,
                     bool tonalRaw, int tf, bool tonalUsed, bool trig) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  display.setCursor(0, 0);
  display.print("peak:");
  display.print((long)peak);
  display.print(" pg:");
  display.print((long)peakGate);

  display.setCursor(0, 12);
  display.print("F:");
  if (bestF > 0) display.print((int)bestF); else display.print("--");
  display.print(" tf:");
  display.print(tf);
  display.print("/");
  display.print(tonalNeedFrames);

  display.setCursor(0, 24);
  display.print("dom:");
  display.print(dom, 2);
  display.print(" pk:");
  display.print(pk, 2);

  display.setCursor(0, 36);
  display.print("loud:");
  display.print(loud ? 1 : 0);
  display.print(" raw:");
  display.print(tonalRaw ? 1 : 0);
  display.print(" used:");
  display.print(tonalUsed ? 1 : 0);

  display.setCursor(0, 48);
  display.print("WiFi:");
  display.print(wifiOk() ? "OK" : "--");
  display.print(" MQTT:");
  display.print(mqttOk() ? "OK" : "--");

  if (trig) {
    display.setTextSize(2);
    display.setCursor(88, 44);
    display.print("TR");
  }

  display.display();
}

// ===================== MAIN =====================
void setup() {
  Serial.begin(115200);
  delay(200);

  for (int i = 0; i < FCNT; i++) bins[i] = makeBin(F[i]);

  Wire.begin(I2C_SDA, I2C_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 init failed");
  } else {
    display.clearDisplay();
    display.display();
  }

  i2sSetup();

  wifiEnsure();
  server.on("/set", handleSet);
  server.on("/status", handleStatus);
  server.begin();

  mqtt.setBufferSize(512);
}

void loop() {
  server.handleClient();

  // WiFi retry (no spam)
  static uint32_t lastWifiTry = 0;
  if (!wifiOk() && millis() - lastWifiTry > 2000) {
    lastWifiTry = millis();
    wifiEnsure();
  }

  // MQTT retry (no spam)
  static uint32_t lastMqttTry = 0;
  if (wifiOk() && !mqttOk() && millis() - lastMqttTry > 2000) {
    lastMqttTry = millis();
    mqttEnsure();
  }
  if (mqttOk()) mqtt.loop();

  // read audio
  size_t bytesRead = 0;
  if (i2s_read(I2S_PORT, rawBuf, sizeof(rawBuf), &bytesRead, pdMS_TO_TICKS(300)) != ESP_OK) return;
  if ((int)(bytesRead / 4) < N) return;

  int32_t peak = 0;
  for (int i = 0; i < N; i++) {
    int32_t v = rawBuf[i] >> SHIFT;
    xBuf[i] = v;
    int32_t a = (v >= 0) ? v : -v;
    if (a > peak) peak = a;
  }

  bool loud = (peak >= peakGate);

  // Goertzel band: best/second/sum
  float best = 0.0f, second = 0.0f, sumBand = 0.0f;
  int bestIdx = -1;

  for (int i = 0; i < FCNT; i++) {
    float m = goertzel_mag2(xBuf, bins[i]);
    sumBand += m;
    if (m > best) { second = best; best = m; bestIdx = i; }
    else if (m > second) { second = m; }
  }

  float bestF = (bestIdx >= 0) ? F[bestIdx] : -1.0f;
  float dom = (second > 0.0f) ? (best / second) : 999.0f;
  float pk  = (sumBand > 0.0f) ? (best / sumBand) : 0.0f;

  // Tonalität (roh) – nur Info
  bool tonalRaw = (dom >= domThr) && (pk >= pkThr);

  // Tonalität verwenden wir NUR, wenn loud==1
  static int tonalFrames = 0;
  if (loud && tonalRaw) tonalFrames++;
  else tonalFrames = 0;

  bool tonalUsed = (tonalFrames >= tonalNeedFrames);

  // Trigger mit cooldown
  static uint32_t cooldownUntil = 0;
  bool trig = loud && tonalUsed && (millis() >= cooldownUntil);

  // UI
  static uint32_t lastUi = 0;
  if (millis() - lastUi > 150) {
    lastUi = millis();
    oledDraw(peak, loud, bestF, dom, pk, tonalRaw, tonalFrames, tonalUsed, trig);
  }

  // Serial
  static uint32_t lastS = 0;
  if (millis() - lastS > 200) {
    lastS = millis();
    Serial.printf("peak=%ld loud=%d bestF=%.0f dom=%.2f pk=%.2f raw=%d tf=%d used=%d\n",
                  (long)peak, loud?1:0, bestF, dom, pk,
                  tonalRaw?1:0, tonalFrames, tonalUsed?1:0);
  }

  if (trig) {
    cooldownUntil = millis() + cooldownMs;
    publishTrig(peak, bestF, dom, pk, tonalFrames);
  }

  // status publish
  static uint32_t lastStat = 0;
  if (millis() - lastStat > 5000) {
    lastStat = millis();
    publishStatus(peak, bestF, dom, pk, tonalFrames);
  }
}
