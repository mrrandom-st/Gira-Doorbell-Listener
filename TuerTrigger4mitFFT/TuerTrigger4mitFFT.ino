#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

#include "driver/i2s.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h> // Für RMS (sqrtf) und Glocken-Animation (sin)

#include "config.h" // Hier liegen WIFI_SSID, WIFI_PASS, MQTT_HOST, etc.

#ifndef WIFI_SSID
#error "Bitte config.h anlegen (Kopie von config.example.h) und WIFI/MQTT Werte setzen."
#endif

// ===================== OLED =====================
#define OLED_W 128
#define OLED_H 64
#define OLED_RESET -1
Adafruit_SSD1306 display(OLED_W, OLED_H, &Wire, OLED_RESET);

// S3-kompatible I2C Pins
#define I2C_SDA 8
#define I2C_SCL 9

// ===================== I2S / INMP441 =====================
#define I2S_WS 4
#define I2S_SCK 5
#define I2S_SD 6
#define I2S_PORT I2S_NUM_0

// ===================== DSP / Trigger =====================
static const float FS = 16000.0f;
static const int N = 512;

// 🔥 DEINE PERFEKTIONIERTEN WERTE 🔥
static int SHIFT = 16;
static int32_t peakGate = 2200;
static uint32_t cooldownMs = 3000;
static int tfNeed = 3;

static float domThr = 1.50f;
static float peakThr = 0.45f;

// ===================== GOERTZEL-FREQUENZEN =====================
static const int FCNT = 9;
static const float F[FCNT] = {
  520, 550, 580, // B1: Sprache/TV
  1600, 1645, 1690, // B2: Klingel Oberton (Index 3, 4, 5)
  2020, 2065, 2110 // B3: Klingel Hauptton (Index 6, 7, 8)
};

// ===================== MQTT Topics =====================
static const char* TOPIC_CFG_PEAKGATE = "home/doorbell/cfg/peakGate";
static const char* TOPIC_CFG_TFNEED = "home/doorbell/cfg/tfNeed";
static const char* TOPIC_CMD_RECORD = "home/doorbell/cmd/record";
static const char* TOPIC_RECORD_DATA = "home/doorbell/record_data";
// (Stelle sicher, dass TOPIC_EVENT und TOPIC_STATUS in der config.h sind!)

WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

// ===================== Recording State =====================
struct RecordEntry {
  int32_t peak;
  float bestF;
  float dom;
  float pk;
  bool tonal;
};

static bool isRecording = false;
static uint32_t recordStartTime = 0;
static uint32_t lastRecordSample = 0;
static RecordEntry recs[20];
static int recCount = 0;

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
    s_prev = s;
  }
  float real = s_prev - s_prev2 * b.cosw;
  float imag = s_prev2 * b.sinw;
  return real * real + imag * imag;
}

// ===================== Buffers =====================
static int32_t rawBuf[N];
static int32_t xBuf[N];
static float mags[FCNT];

// ===================== Runtime values =====================
static int32_t lastPeak = 0;
static float lastBestF = -1.0f;
static float lastDom = 0.0f;
static float lastPk = 0.0f;
static bool lastTrig = false;

// ===== Klingel-Pattern =====
static int toneHistory[6] = {0};
static int tonePos = 0;

// ===================== Helpers =====================
static bool wifiOk() { return WiFi.status() == WL_CONNECTED; }
static bool mqttOk() { return mqtt.connected(); }

static void wifiEnsure() {
  if (!wifiOk()) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
  }
}

static void publishStatus() {
  if (!mqttOk()) return;
  char buf[320];
  snprintf(buf, sizeof(buf),
    "{\"peak\":%ld,\"bestF\":%.0f,\"dom\":%.2f,\"pk\":%.2f,\"peakGate\":%ld,\"tfNeed\":%d,\"cooldownMs\":%u}",
    (long)lastPeak, lastBestF, lastDom, lastPk, (long)peakGate, tfNeed, (unsigned)cooldownMs);
  mqtt.publish(TOPIC_STATUS, buf, true);
}

static void publishTrig() {
  if (!mqttOk()) return;
  char buf[128];
  snprintf(buf, sizeof(buf), "{\"trigger\":1,\"peak\":%ld,\"freq\":%.0f}", (long)lastPeak, lastBestF);
  mqtt.publish(TOPIC_EVENT, buf, false);
}

static void sendRecordData() {
  if (!mqttOk()) return;
  String json = "[";
  for (int i = 0; i < recCount; i++) {
    char buf[128];
    snprintf(buf, sizeof(buf), "{\"p\":%ld,\"f\":%.0f,\"d\":%.2f,\"k\":%.2f,\"t\":%d}",
      recs[i].peak, recs[i].bestF, recs[i].dom, recs[i].pk, recs[i].tonal ? 1 : 0);
    json += buf;
    if (i < recCount - 1) json += ",";
  }
  json += "]";
  mqtt.publish(TOPIC_RECORD_DATA, json.c_str(), false);
  mqtt.publish(TOPIC_CMD_RECORD, "0", true);
}

static void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String v = "";
  for (unsigned int i = 0; i < length; i++) v += (char)payload[i];
  String t = String(topic);
  int val = v.toInt();

  // START RECORDING
  if (t == TOPIC_CMD_RECORD && val == 1 && !isRecording) {
    isRecording = true;
    recordStartTime = millis();
    lastRecordSample = 0;
    recCount = 0;
    return;
  }

  // 🔥 MQTT TÜRSTEHER (Sichert deine perfekten Werte) 🔥
  if (t == TOPIC_CFG_PEAKGATE) {
    if (val >= 1000 && val <= 10000) {
      peakGate = val;
      publishStatus();
    }
  } else if (t == TOPIC_CFG_TFNEED) {
    if (val >= 1 && val <= 10) {
      tfNeed = val;
      publishStatus();
    }
  }
}

static void mqttEnsure() {
  if (!wifiOk() || mqttOk()) return;
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  String clientId = "doorbell-" + String((uint32_t)ESP.getEfuseMac(), HEX);

  if (mqtt.connect(clientId.c_str(), MQTT_USER, MQTT_PASS)) {
    mqtt.subscribe(TOPIC_CFG_PEAKGATE);
    mqtt.subscribe(TOPIC_CFG_TFNEED);
    mqtt.subscribe(TOPIC_CMD_RECORD);
    publishStatus(); // Sofort korrekte Werte an ioBroker melden
  }
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
  cfg.dma_buf_len = N;

  i2s_pin_config_t pin = {};
  pin.bck_io_num = I2S_SCK;
  pin.ws_io_num = I2S_WS;
  pin.data_out_num = -1;
  pin.data_in_num = I2S_SD;

  i2s_driver_install(I2S_PORT, &cfg, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin);
  i2s_set_clk(I2S_PORT, (uint32_t)FS, I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_MONO);
  i2s_start(I2S_PORT);
}

// ===================== OLED ANIMATIONEN =====================
static void drawBellAnimation() {
  display.clearDisplay();

  int cx = 64;
  int cy = 28;

  int swing = sin(millis() / 50.0) * 12;

  if ((millis() / 150) % 2 == 0) {
    display.drawCircle(cx, cy + 4, 30, SSD1306_WHITE);
    display.drawCircle(cx, cy + 4, 35, SSD1306_WHITE);
    display.drawCircle(cx, cy + 4, 40, SSD1306_WHITE);
  }

  display.fillRect(cx - 24, 0, 49, 64, SSD1306_BLACK);
  display.fillCircle(cx + swing, cy + 24, 7, SSD1306_WHITE);

  display.fillCircle(cx, cy, 18, SSD1306_WHITE);
  display.fillRect(cx - 18, cy, 37, 16, SSD1306_WHITE);
  display.fillRoundRect(cx - 24, cy + 12, 49, 10, 4, SSD1306_WHITE);
  display.drawCircle(cx, cy - 22, 4, SSD1306_WHITE);

  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  if ((millis() / 250) % 2 == 0) {
    display.setCursor(2, 2);
    display.print("DING");
  } else {
    display.setCursor(76, 48);
    display.print("DONG");
  }

  display.display();
}

static void oledDrawNormal() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  display.setCursor(0, 0);
  display.printf("Vol:%4ld Gate:%ld", (long)lastPeak, (long)peakGate);
  display.setCursor(0, 12);
  display.printf("Hz: %4.0f Dom:%.1f", lastBestF, lastDom);

  if (isRecording) {
    display.fillRect(100, 16, 28, 14, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
    display.setCursor(105, 19);
    display.print("REC");
  }

  int barWidth = 10;
  int startX = 10;
  float maxMag = 0.1f;
  for (int i = 0; i < FCNT; i++) if (mags[i] > maxMag) maxMag = mags[i];

  for (int i = 0; i < FCNT; i++) {
    int barHeight = (int)((mags[i] / maxMag) * 25.0f);
    if (barHeight > 25) barHeight = 25;
    if (barHeight < 1) barHeight = 1;
    display.fillRect(startX + (i * 12), 64 - barHeight, barWidth, barHeight, SSD1306_WHITE);
  }

  display.display();
}

// ===================== DSP Analyse =====================
static void analyzeAudio() {
  size_t bytesRead = 0;
  if (i2s_read(I2S_PORT, rawBuf, sizeof(rawBuf), &bytesRead, pdMS_TO_TICKS(300)) != ESP_OK) return;
  if ((int)(bytesRead / 4) < N) return;

  int32_t peak = 0;
  float rms = 0.0f;

  for (int i = 0; i < N; i++) {
    int32_t v = rawBuf[i] >> SHIFT;
    xBuf[i] = v;

    int32_t a = (v >= 0) ? v : -v;
    if (a > peak) peak = a;

    rms += (float)v * v;
  }

  rms = sqrtf(rms / N);

  float best = 0.0f, second = 0.0f, sumBand = 0.0f;
  int bestIdx = -1;

  for (int i = 0; i < FCNT; i++) {
    mags[i] = goertzel_mag2(xBuf, bins[i]);
    sumBand += mags[i];

    if (mags[i] > best) {
      second = best;
      best = mags[i];
      bestIdx = i;
    } else if (mags[i] > second) {
      second = mags[i];
    }
  }

  float bestF = (bestIdx >= 0) ? F[bestIdx] : -1.0f;

  // ===== Klingel-Töne erkennen =====
  bool isLowTone = (bestIdx == 4 || bestIdx == 3); // 1600 / 1645
  bool isHighTone = (bestIdx == 7 || bestIdx == 6); // 2065 / 2020

  int tone = 0;
  if (isLowTone) tone = 1;
  else if (isHighTone) tone = 2;

  // History speichern
  toneHistory[tonePos] = tone;
  tonePos = (tonePos + 1) % 6;

  // 🔥 Stabile Dominanz (Division-by-Zero Schutz)
  float dom = (second > 1e-6f) ? (best / second) : 50.0f;
  if (dom > 50.0f) dom = 50.0f;

  float pk = (sumBand > 0.0f) ? (best / sumBand) : 0.0f;

  // 🔥 Crest-Faktor (Crest = Peak / RMS)
  float crest = (rms > 1e-6f) ? ((float)peak / rms) : 999.0f;

  // 🔥 KORRIGIERT: Ton statt Knall erkennen
  bool loud = (peak >= peakGate) && (crest < 2.5f);

  // 🔥 Bin-basierte Frequenzprüfung (Ignoriert Sprache 520-580 Hz)
  bool freqOk = (bestIdx >= 3);

  bool tonalOk = (dom >= domThr) && (pk >= peakThr);

  // ===== Pattern prüfen =====
  bool seenLow = false;
  bool seenHigh = false;

  for (int i = 0; i < 6; i++) {
    if (toneHistory[i] == 1) seenLow = true;
    if (toneHistory[i] == 2) seenHigh = true;
  }

  bool hasPattern = seenLow && seenHigh;

  // 🔥 Leaky Bucket (Weicher tfStreak)
  static int tfStreak = 0;
  if (loud && freqOk && tonalOk && hasPattern) {
    tfStreak++;
  } else if (tfStreak > 0) {
    tfStreak--;
  }

  static uint32_t lastTriggerMs = 0;
  bool cooldownReady = (lastTriggerMs == 0) || (millis() - lastTriggerMs >= cooldownMs);
  bool trig = (tfStreak >= tfNeed) && cooldownReady;

  if (trig) {
    lastTriggerMs = millis();
    tfStreak = 0;
    lastTrig = true;
    publishTrig();
  }

  lastPeak = peak;
  lastBestF = bestF;
  lastDom = dom;
  lastPk = pk;

  // 🔥 RECORDING
  if (isRecording) {
    if (millis() - recordStartTime >= 5000) {
      isRecording = false;
      sendRecordData();
    } else {
      if (millis() - lastRecordSample >= 250) {
        lastRecordSample = millis();
        if (recCount < 20) {
          recs[recCount].peak = peak;
          recs[recCount].bestF = bestF;
          recs[recCount].dom = dom;
          recs[recCount].pk = pk;
          recs[recCount].tonal = (freqOk && tonalOk);
          recCount++;
        }
      }
    }
  }
}

// ===================== Setup / Loop =====================
void setup() {
  Serial.begin(115200);
  delay(1000);

  for (int i = 0; i < FCNT; i++) bins[i] = makeBin(F[i]);

  // Turbo I2C (400kHz) für flüssige Animation
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) Serial.println("OLED fail");
  else {
    display.clearDisplay();
    display.display();
  }

  i2sSetup();
  mqtt.setCallback(mqttCallback);
  mqtt.setBufferSize(2048);

  wifiEnsure();
}

void loop() {
  static uint32_t lastNetTry = 0;
  if (millis() - lastNetTry > 3000) {
    lastNetTry = millis();
    wifiEnsure();
    mqttEnsure();
  }

  if (mqttOk()) mqtt.loop();

  analyzeAudio();

  // ================= DISPLAY STEUERUNG =================
  static uint32_t lastUi = 0;
  uint32_t uiDelay = lastTrig ? 40 : 150; // Turbo-FPS bei Animation

  if (millis() - lastUi > uiDelay) {
    lastUi = millis();
    if (lastTrig) {
      drawBellAnimation();
    } else {
      oledDrawNormal();
    }
  }

  // ================= MQTT STATUS (Alle 5s) =================
  static uint32_t lastStat = 0;
  if (millis() - lastStat > 5000) {
    lastStat = millis();
    publishStatus();
  }

  // ================= ANIMATIONS TIMER =================
  static uint32_t trigShownAt = 0;
  if (lastTrig) {
    if (trigShownAt == 0) trigShownAt = millis();
    if (millis() - trigShownAt > 4000) { // 4 Sekunden bimmeln
      lastTrig = false;
      trigShownAt = 0;
    }
  } else {
    trigShownAt = 0;
  }
}
