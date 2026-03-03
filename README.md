# Gira Doorbell Listener

Arduino/ESP32-Projekt zur Erkennung einer **Gira-Türklingel** über ein Mikrofon (INMP441), Versand von Trigger-/Statusdaten über MQTT und Anzeige auf einem M5Stack CoreS3.

## Projektüberblick

Dieses Repository enthält mehrere Sketch-Varianten für unterschiedliche Entwicklungsstände:

- `TuerTrigger2/` – robuster Trigger-Test mit Goertzel-Bändern, HOLD-Logik und MQTT-Status.
- `TuerTrigger3mitFFT/` – Trigger-/Analyse-Variante mit Fokus auf FFT/Goertzel-Tuning sowie Web-Endpunkte zum schnellen Nachstellen.
- `CorsS3EmpfangKlingel1/` – CoreS3-Empfänger/Anzeige mit JSON-Parsing über ArduinoJson.
- `CoreS3-GPT-Listener1/` – überarbeiteter CoreS3-Empfänger mit robusterem Parsing, non-blocking Alert-Sound und Display-Optimierungen.

## Hardware (typisch)

- ESP32-Board für Trigger (z. B. ESP32 DevKit)
- INMP441 I2S-Mikrofon
- optional OLED 128x64 (SSD1306) für lokale Messwerte
- M5Stack CoreS3 (Display/Empfang)
- WLAN + MQTT-Broker (z. B. Mosquitto)

## Software-Voraussetzungen

- Arduino IDE **oder** PlatformIO
- ESP32-Boardpaket
- Bibliotheken (je nach Sketch):
  - `PubSubClient`
  - `ArduinoJson` (für `CorsS3EmpfangKlingel1`)
  - `M5Unified` (für CoreS3-Sketches)
  - `Adafruit GFX` + `Adafruit SSD1306` (für Trigger-Sketches mit OLED)

## Konfiguration

Jeder Sketch nutzt eine lokale `config.h`.

1. In jedem benötigten Sketch-Ordner die Datei `config.example.h` nach `config.h` kopieren.
2. Werte für WLAN, MQTT und Topics setzen.

Beispiel:

```cpp
static const char* WIFI_SSID = "DEIN_WIFI_NAME";
static const char* WIFI_PASS = "DEIN_WIFI_PASSWORT";

static const char* MQTT_HOST = "192.168.178.10";
static const uint16_t MQTT_PORT = 1885;
static const char* MQTT_USER = "mqtt_user";
static const char* MQTT_PASS = "mqtt_passwort";

static const char* TOPIC_EVENT  = "home/doorbell/event";
static const char* TOPIC_STATUS = "home/doorbell/status";
```

> Hinweis: `config.h` enthält Zugangsdaten und sollte nicht ins Repository committed werden.

## Nutzung (empfohlener Ablauf)

1. **Trigger-Sketch** auf ESP32 flashen (`TuerTrigger2` oder `TuerTrigger3mitFFT`).
2. Prüfen, ob regelmäßig `TOPIC_STATUS` gesendet wird.
3. **Empfänger-Sketch** auf CoreS3 flashen (`CoreS3-GPT-Listener1` empfohlen).
4. Bei Klingeln sollte ein Event auf `TOPIC_EVENT` erscheinen und auf dem CoreS3 ein Alarm angezeigt werden.

## MQTT-Daten (vereinfacht)

### Event

Typischer Inhalt (je nach Sketch leicht unterschiedlich):

```json
{
  "trigger": 1,
  "peak": 1234,
  "bestF": 1645,
  "dom": 1.8,
  "pk": 0.34,
  "tf": 5
}
```

### Status

Typischer Inhalt:

```json
{
  "online": 1,
  "peakGate": 3000,
  "domThr": 1.45,
  "pkThr": 0.30,
  "tfNeed": 5,
  "cool": 1500,
  "shift": 16,
  "ip": "192.168.x.x"
}
```

## Tipps zum Tuning

- `peakGate` höher setzen, wenn zu viele Fehltrigger auftreten.
- `domThr`/`pkThr` anpassen, um „tonale“ Klingelsignale besser von Klatschen/Impulsen zu unterscheiden.
- `tfNeed` erhöhen, wenn Trigger stabiler (aber träger) werden soll.
- Bei `TuerTrigger3mitFFT` können Parameter über den Webserver schnell geändert werden (siehe Sketch-Kommentare).

## Lizenz

Derzeit ist keine explizite Lizenzdatei enthalten. Bei Bedarf bitte `LICENSE` ergänzen.
