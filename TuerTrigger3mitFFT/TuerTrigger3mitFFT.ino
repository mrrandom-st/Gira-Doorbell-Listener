#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <PubSubClient.h>

#include "driver/i2s.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ===================== USER CONFIG =====================
static const char* WIFI_SSID = "";
static const char* WIFI_PASS = "";

static const char* MQTT_HOST = "";
static const uint16_t MQTT_PORT = 1885;
static const char* MQTT_USER = "";
static const char* MQTT_PASS = "";

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
static const int   N  = 512;

// Lautstärke: eher „niedrig empfindlich“ = höherer Gate-Wert
static int   SHIFT      = 16;
static int32_t peakGate = 450;     // <- deine gute Grundlage beibehalten
static uint32_t cooldownMs = 1200; // bleibt drin (für später), wird jetzt nicht genutzt

// ===================== GOERTZEL-FREQUENZEN (NUR RELEVANT GEÄNDERT) =====================
// Aus FFT: dominante Bereiche ~550 Hz, ~1645 Hz (stärkster), ~2065 Hz.
// Wir nehmen je Bereich ein kleines Band (± ~30..50 Hz), damit Drift/Verzerrung nicht stört.
static const int FCNT = 9;
static const float F[FCNT] = {
  520, 550, 580,      // Band A ~550
  1600, 1645, 1690,   // Band B ~1645 (Hauptton)
  2020, 2065, 2110    // Band C ~2065
};

// Diese Schwellen bleiben als "Tuning-Parameter" drin, aber Trigger wird unten deaktiviert
static float domThr  = 1.70f;
static float peakThr = 0.24f;

// ===================== Networking =====================
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);
WebServer server(80);

// ===================== Goertzel =====================
struct GoertzelBin {
  float coeff;
  float cosw;
  float sinw;
};
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

// ===================== Audio buffers =====================
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

// publishTrig bleibt drin (für später), wird aber unten NICHT aufgerufen
static void publishTrig(int32_t peak, float best, float second, float dom, float pk, float bestF) {
  if (!mqttOk()) return;
  char buf[256];
  snprintf(buf, sizeof(buf),
           "{\"trigger\":1,\"peak\":%ld,\"bestF\":%.0f,\"best\":%.0f,\"second\":%.0f,\"dom\":%.2f,\"pk\":%.2f}",
           (long)peak, bestF, best, second, dom, pk);
  mqtt.publish(TOPIC_EVENT, buf, false);
}

static void publishStatus(int32_t peak, float best, float second, float dom, float pk, float bestF) {
  if (!mqttOk()) return;
  char buf[512];
  snprintf(buf, sizeof(buf),
           "{\"online\":1,"
           "\"peakGate\":%ld,\"domThr\":%.2f,\"peakThr\":%.2f,"
           "\"peak\":%ld,\"bestF\":%.0f,\"best\":%.0f,\"second\":%.0f,\"dom\":%.2f,\"pk\":%.2f,"
           "\"ip\":\"%s\"}",
           (long)peakGate, domThr, peakThr,
           (long)peak, bestF, best, second, dom, pk,
           wifiOk()?WiFi.localIP().toString().c_str():"--");
  mqtt.publish(TOPIC_STATUS, buf, true);
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

// ===================== Web: quick tuning =====================
// /set?pg=500&dom=1.8&pk=0.28&cd=1200
static void handleSet() {
  if (server.hasArg("pg"))  peakGate = server.arg("pg").toInt();
  if (server.hasArg("dom")) domThr   = server.arg("dom").toFloat();
  if (server.hasArg("pk"))  peakThr  = server.arg("pk").toFloat();
  if (server.hasArg("cd"))  cooldownMs = (uint32_t)server.arg("cd").toInt();

  String s = "OK ";
  s += "pg=" + String((long)peakGate);
  s += " dom=" + String(domThr, 2);
  s += " pk=" + String(peakThr, 2);
  s += " cd=" + String((unsigned)cooldownMs);
  server.send(200, "text/plain", s);
}

static void handleStatus() {
  String s = "{";
  s += "\"wifi\":" + String(wifiOk()?1:0) + ",";
  s += "\"mqtt\":" + String(mqttOk()?1:0) + ",";
  s += "\"peakGate\":" + String((long)peakGate) + ",";
  s += "\"domThr\":" + String(domThr, 2) + ",";
  s += "\"peakThr\":" + String(peakThr, 2) + ",";
  s += "\"cooldownMs\":" + String((unsigned)cooldownMs);
  s += "}";
  server.send(200, "application/json", s);
}

// ===================== OLED draw =====================
static void oledDraw(int32_t peak, bool loud, float bestF, float dom, float pk, bool tonalOk, bool trig) {
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
  display.print(" dom:");
  display.print(dom, 2);

  display.setCursor(0, 24);
  display.print("pk:");
  display.print(pk, 2);
  display.print(" thr:");
  display.print(peakThr, 2);

  display.setCursor(0, 36);
  display.print("loud:");
  display.print(loud ? 1 : 0);
  display.print(" tonal:");
  display.print(tonalOk ? 1 : 0);

  display.setCursor(0, 48);
  display.print("WiFi:");
  display.print(wifiOk() ? "OK" : "--");
  display.print(" MQTT:");
  display.print(mqttOk() ? "OK" : "--");

  // trig wird aktuell immer false gelassen (kein TR anzeigen)
  if (trig) {
    display.setTextSize(2);
    display.setCursor(64, 44);
    display.print("TR");
  }

  display.display();
}

// ===================== Setup / Loop =====================
void setup() {
  Serial.begin(115200);
  delay(200);

  // precompute bins
  for (int i = 0; i < FCNT; i++) bins[i] = makeBin(F[i]);

  // OLED init
  Wire.begin(I2C_SDA, I2C_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 init failed");
  } else {
    display.clearDisplay();
    display.display();
  }

  // I2S
  i2sSetup();

  // Networking
  wifiEnsure();
  server.on("/set", handleSet);
  server.on("/status", handleStatus);
  server.begin();

  mqtt.setBufferSize(512);
}

void loop() {
  // Web
  server.handleClient();

  // WiFi: try only every 2s (no spam)
  static uint32_t lastWifiTry = 0;
  if (!wifiOk() && millis() - lastWifiTry > 2000) {
    lastWifiTry = millis();
    wifiEnsure();
  }

  // MQTT: try only every 2s
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

  // Goertzel: find best/second + sumBand (jetzt über sinnvolle Frequenzen)
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

  // tonalOk wird nur angezeigt – keine Erkennung/Trigger
  bool tonalOk = (dom >= domThr) && (pk >= peakThr);

  // ===================== TRIGGER DEAKTIVIERT (NUR GRUNDLAGE) =====================
  bool trig = false; // <- absichtlich aus

  // OLED
  static uint32_t lastUi = 0;
  if (millis() - lastUi > 150) {
    lastUi = millis();
    oledDraw(peak, loud, bestF, dom, pk, tonalOk, trig);
  }

  // Serial (every 200ms)
  static uint32_t lastS = 0;
  if (millis() - lastS > 200) {
    lastS = millis();
    Serial.printf("peak=%ld loud=%d bestF=%.0f dom=%.2f pk=%.2f tonal=%d\n",
                  (long)peak, loud?1:0, bestF, dom, pk, tonalOk?1:0);
  }

  // ===================== KEIN publishTrig() =====================
  // (für später: hier würden wir triggern und cooldown nutzen)

  // status publish (5s) – bleibt hilfreich zum Einpegeln
  static uint32_t lastStat = 0;
  if (millis() - lastStat > 5000) {
    lastStat = millis();
    publishStatus(peak, best, second, dom, pk, bestF);
  }
}
