#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <PubSubClient.h>
#include "driver/i2s.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

static const char* WIFI_SSID = "R2DLan";
static const char* WIFI_PASS = "#Stregner18!";
static const char* MQTT_HOST = "192.168.2.142";
static const uint16_t MQTT_PORT = 1885;
static const char* MQTT_USER = "randommqtt";
static const char* MQTT_PASS = "cH1Ll0U7";
static const char* TOPIC_EVENT  = "home/doorbell/event";
static const char* TOPIC_STATUS = "home/doorbell/status";

#define OLED_W 128
#define OLED_H 64
#define OLED_RESET -1
Adafruit_SSD1306 display(OLED_W, OLED_H, &Wire, OLED_RESET);
#define I2C_SDA 21
#define I2C_SCL 22
#define I2S_WS   25
#define I2S_SCK  32
#define I2S_SD   33
#define I2S_PORT I2S_NUM_0

static const float FS = 16000.0f;
static const int N = 512;
static int SHIFT = 16;
static int32_t peakGate = 3000;
static uint32_t cooldownMs = 1500;
static const int BAND_COUNT = 3;
static const int BINS_PER_BAND = 3;
static const float BAND_CENTER[3] = { 545.0f, 1645.0f, 2065.0f };
static const int TOTAL_BINS = 9;
static float F[9];
static float bandRatioMin = 0.15f;
static float bandDomMin = 1.3f;
static int tonalNeedFrames = 5;

WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);
WebServer server(80);

struct GoertzelBin { float coeff, cosw, sinw; };
struct BandResult { float energy[3]; float totalBand; float totalAll; float minRatio; float dominance; };
static GoertzelBin bins[9];
static int32_t rawBuf[512];
static int32_t xBuf[512];

static GoertzelBin makeBin(float f0) {
  int k = (int)(0.5f + (512 * f0) / 16000.0f);
  float w = 2.0f * 3.14159265f * (float)k / 512.0f;
  GoertzelBin b;
  b.cosw = cosf(w);
  b.sinw = sinf(w);
  b.coeff = 2.0f * b.cosw;
  return b;
}

static float goertzel_mag2(const int32_t* x, const GoertzelBin& b) {
  float s1 = 0.0f, s2 = 0.0f;
  for (int i = 0; i < 512; i++) {
    float s = (float)x[i] + b.coeff * s1 - s2;
    s2 = s1;
    s1 = s;
  }
  float real = s1 - s2 * b.cosw;
  float imag = s2 * b.sinw;
  return real * real + imag * imag;
}

static BandResult analyzeBands(const int32_t* x) {
  BandResult r = {};
  float mag[9];
  for (int i = 0; i < 9; i++) {
    mag[i] = goertzel_mag2(x, bins[i]);
    r.totalAll += mag[i];
  }
  for (int band = 0; band < 3; band++) {
    for (int bin = 0; bin < 3; bin++) {
      int idx = band * 3 + bin;
      r.energy[band] += mag[idx];
    }
    r.totalBand += r.energy[band];
  }
  r.minRatio = 999.0f;
  if (r.totalBand > 0.0f) {
    for (int band = 0; band < 3; band++) {
      float ratio = r.energy[band] / r.totalBand;
      if (ratio < r.minRatio) r.minRatio = ratio;
    }
  }
  r.dominance = (r.totalAll > 0.0f) ? (r.totalBand / r.totalAll) : 0.0f;
  return r;
}

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
  if (!wifiOk() || mqttOk()) return;
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  String cid = "doorbell-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  mqtt.connect(cid.c_str(), MQTT_USER, MQTT_PASS);
}

static void i2sSetup() {
  i2s_config_t cfg = {};
  cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
  cfg.sample_rate = 16000;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;
  cfg.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
  cfg.communication_format = (i2s_comm_format_t)I2S_COMM_FORMAT_STAND_I2S;
  cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  cfg.dma_buf_count = 8;
  cfg.dma_buf_len = 512;
  i2s_pin_config_t pin = {};
  pin.bck_io_num = 32;
  pin.ws_io_num = 25;
  pin.data_out_num = -1;
  pin.data_in_num = 33;
  i2s_driver_install(I2S_PORT, &cfg, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin);
  i2s_set_clk(I2S_PORT, 16000, I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_MONO);
  i2s_zero_dma_buffer(I2S_PORT);
  i2s_start(I2S_PORT);
}

static void publishTrig(int32_t p, float b1, float b2, float b3, float r, int tf) {
  if (!mqttOk()) return;
  char buf[256];
  snprintf(buf, 256, "{\"trigger\":1,\"peak\":%ld,\"b1\":%.0f,\"b2\":%.0f,\"b3\":%.0f,\"ratio\":%.2f,\"tf\":%d}", (long)p, b1, b2, b3, r, tf);
  mqtt.publish(TOPIC_EVENT, buf, false);
}

static void publishStatus(int32_t p, float b1, float b2, float b3, float r, int tf) {
  if (!mqttOk()) return;
  String ip = wifiOk() ? WiFi.localIP().toString() : "--";
  char buf[512];
  snprintf(buf, 512, "{\"online\":1,\"peakGate\":%ld,\"bandRatioMin\":%.2f,\"bandDomMin\":%.2f,\"tfNeed\":%d,\"cool\":%u,\"shift\":%d,\"peak\":%ld,\"b1\":%.0f,\"b2\":%.0f,\"b3\":%.0f,\"ratio\":%.2f,\"tf\":%d,\"ip\":\"%s\"}", (long)peakGate, bandRatioMin, bandDomMin, tonalNeedFrames, (unsigned)cooldownMs, SHIFT, (long)p, b1, b2, b3, r, tf, ip.c_str());
  mqtt.publish(TOPIC_STATUS, buf, true);
}
static void handleSet() {
  if (server.hasArg("pg")) peakGate = server.arg("pg").toInt();
  if (server.hasArg("ratio")) bandRatioMin = server.arg("ratio").toFloat();
  if (server.hasArg("dom")) bandDomMin = server.arg("dom").toFloat();
  if (server.hasArg("tf")) {
    int val = server.arg("tf").toInt();
    tonalNeedFrames = (val > 2) ? val : 2;
  }
  if (server.hasArg("cd")) cooldownMs = server.arg("cd").toInt();
  if (server.hasArg("shift")) {
    int val = server.arg("shift").toInt();
    SHIFT = (val < 10) ? 10 : (val > 20) ? 20 : val;
  }
  String s = "OK pg=" + String((long)peakGate) + " ratio=" + String(bandRatioMin, 2) + " dom=" + String(bandDomMin, 2) + " tf=" + String(tonalNeedFrames) + " cd=" + String((unsigned)cooldownMs) + " shift=" + String(SHIFT);
  server.send(200, "text/plain", s);
}

static void handleStatus() {
  String s = "{\"wifi\":" + String(wifiOk()?1:0) + ",\"mqtt\":" + String(mqttOk()?1:0) + ",\"peakGate\":" + String((long)peakGate) + ",\"bandRatioMin\":" + String(bandRatioMin, 2) + ",\"bandDomMin\":" + String(bandDomMin, 2) + ",\"tfNeed\":" + String(tonalNeedFrames) + ",\"cooldownMs\":" + String((unsigned)cooldownMs) + ",\"shift\":" + String(SHIFT) + "}";
  server.send(200, "application/json", s);
}

static void oledDraw(int32_t peak, bool loud, float b1, float b2, float b3, float ratio, bool tonalRaw, int tf, bool tonalUsed, bool trig) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("peak:");
  display.print((long)peak);
  display.print(" pg:");
  display.print((long)peakGate);
  display.setCursor(0, 12);
  display.print("B1:");
  display.print((int)b1);
  display.print(" B2:");
  display.print((int)b2);
  display.print(" B3:");
  display.print((int)b3);
  display.setCursor(0, 24);
  display.print("ratio:");
  display.print(ratio, 2);
  display.print(" tf:");
  display.print(tf);
  display.print("/");
  display.print(tonalNeedFrames);
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

void setup() {
  Serial.begin(115200);
  delay(200);
  float bw = 25.0f;
  for (int band = 0; band < 3; band++) {
    float center = BAND_CENTER[band];
    for (int bin = 0; bin < 3; bin++) {
      int idx = band * 3 + bin;
      F[idx] = center + (bin - 1) * bw;
      bins[idx] = makeBin(F[idx]);
    }
  }
  Serial.println("=== Goertzel Frequency Bands ===");
  for (int band = 0; band < 3; band++) {
    Serial.printf("Band %d (%.0f Hz): ", band+1, BAND_CENTER[band]);
    for (int bin = 0; bin < 3; bin++) {
      int idx = band * 3 + bin;
      Serial.printf("%.0f ", F[idx]);
    }
    Serial.println();
  }
  Wire.begin(21, 22);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED fail");
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
  static uint32_t lastW = 0;
  if (!wifiOk() && millis() - lastW > 2000) {
    lastW = millis();
    wifiEnsure();
  }
  static uint32_t lastM = 0;
  if (wifiOk() && !mqttOk() && millis() - lastM > 2000) {
    lastM = millis();
    mqttEnsure();
  }
  if (mqttOk()) mqtt.loop();
  size_t br = 0;
  if (i2s_read(I2S_PORT, rawBuf, sizeof(rawBuf), &br, pdMS_TO_TICKS(300)) != ESP_OK) return;
  if ((int)(br / 4) < 512) return;
  int32_t peak = 0;
  for (int i = 0; i < 512; i++) {
    int32_t v = rawBuf[i] >> SHIFT;
    xBuf[i] = v;
    int32_t a = (v >= 0) ? v : -v;
    if (a > peak) peak = a;
  }
  bool loud = (peak >= peakGate);
  BandResult b = analyzeBands(xBuf);
  bool tonalRaw = (b.minRatio >= bandRatioMin) && (b.dominance >= bandDomMin);
  static int tonalFrames = 0;
  if (loud && tonalRaw) tonalFrames++;
  else tonalFrames = 0;
  bool tonalUsed = (tonalFrames >= tonalNeedFrames);
  static uint32_t cooldownUntil = 0;
  bool trig = loud && tonalUsed && (millis() >= cooldownUntil);
  static uint32_t lastUi = 0;
  if (millis() - lastUi > 150) {
    lastUi = millis();
    oledDraw(peak, loud, b.energy[0], b.energy[1], b.energy[2], b.minRatio, tonalRaw, tonalFrames, tonalUsed, trig);
  }
  static uint32_t lastS = 0;
  if (millis() - lastS > 200) {
    lastS = millis();
    Serial.printf("peak=%ld loud=%d B1=%.0f B2=%.0f B3=%.0f ratio=%.2f dom=%.2f raw=%d tf=%d used=%d\n", (long)peak, loud?1:0, b.energy[0], b.energy[1], b.energy[2], b.minRatio, b.dominance, tonalRaw?1:0, tonalFrames, tonalUsed?1:0);
  }
  if (trig) {
    cooldownUntil = millis() + cooldownMs;
    publishTrig(peak, b.energy[0], b.energy[1], b.energy[2], b.minRatio, tonalFrames);
  }
  static uint32_t lastStat = 0;
  if (millis() - lastStat > 5000) {
    lastStat = millis();
    publishStatus(peak, b.energy[0], b.energy[1], b.energy[2], b.minRatio, tonalFrames);
  }
}
