#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ArduinoJson.h>

// --- Pin Definitions ---
#define LORA_CS     10
#define LORA_RST    9
#define LORA_DIO0   2

#define BTN_POWER    3
#define BTN_GUARDIAN 4
#define BTN_MONITOR  5

#define LORA_FREQ    868E6  // Frequency (433E6, 868E6, 915E6)
#define DEBOUNCE_MS  200

#define EMA_ALPHA    0.3f   // Smoothing factor for receiver-side dynamic averages

// --- Hardware Objects ---
LiquidCrystal_I2C lcd(0x27, 16, 2); 

// --- System Telemetry State Structure ---
struct BaseStationState {
  // Raw 30-sec window batch averages (from ESP32 transmitter)
  float temp = 0.0;
  float hum = 0.0;
  int voc = 0;
  unsigned int sampleCount = 0;

  // Local Receiver-side Exponential Moving Averages (EMA)
  float emaTemp = 0.0;
  float emaHum = 0.0;
  float emaVoc = 0.0;

  char mode[10] = "INIT";
  bool breach = false;
  bool lastDataValid = false;
  unsigned long lastRxTime = 0;
  int rssi = 0;             // dBm of last received packet
  float snr = 0.0;          // dB
  unsigned int packets = 0; // Received since boot
} baseState;

// --- Button Debounce Tracking ---
unsigned long lastPressTimePower = 0;
unsigned long lastPressTimeGuardian = 0;
unsigned long lastPressTimeMonitor = 0;

// --- Function Declarations ---
void sendLoRaCommand(const char* action, const char* value);
void parseIncomingLoRa();
void updateDisplay();
void handleButtons();
void handleSerialCommands();

void setup() {
  Serial.begin(115200);

  // Initialize Push Buttons with internal pullups
  pinMode(BTN_POWER, INPUT_PULLUP);
  pinMode(BTN_GUARDIAN, INPUT_PULLUP);
  pinMode(BTN_MONITOR, INPUT_PULLUP);

  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Ground Station ");
  lcd.setCursor(0, 1);
  lcd.print("Init LoRa...    ");

  // Initialize LoRa Transceiver
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println(F("Error: LoRa initialization failed!"));
    lcd.setCursor(0, 1);
    lcd.print("LoRa INIT FAIL!");
    while (1);
  }

  // Radio Configurations
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.enableCrc();

  Serial.println(F("Ground Station initialized successfully."));
  lcd.setCursor(0, 1);
  lcd.print("Waiting Base... ");
  delay(1500);
  lcd.clear();
}

void loop() {
  handleButtons();
  handleSerialCommands();
  parseIncomingLoRa();
  updateDisplay();
}

// --- USB Command Handler -----------------------------------------------------
void handleSerialCommands() {
  static char buf[64];
  static byte len = 0;

  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (len == 0) continue;
      buf[len] = '\0';
      len = 0;

      StaticJsonDocument<96> doc;
      if (deserializeJson(doc, buf)) {
        Serial.println(F("Serial cmd: bad JSON"));
        continue;
      }

      const char* cmd = doc["cmd"];
      const char* val = doc["val"] | "";
      if (!cmd) {
        Serial.println(F("Serial cmd: missing 'cmd'"));
        continue;
      }

      Serial.print(F("Serial cmd -> "));
      Serial.print(cmd);
      Serial.print(' ');
      Serial.println(val);

      sendLoRaCommand(cmd, val);
    }
    else if (len < sizeof(buf) - 1) {
      buf[len++] = c;
    }
    else {
      len = 0; // Discard overlong lines
    }
  }
}

// --- Button Handler Routine ---
void handleButtons() {
  unsigned long now = millis();

  if (digitalRead(BTN_POWER) == LOW) {
    if (now - lastPressTimePower > DEBOUNCE_MS) {
      Serial.println(F("BTN: Power Toggle Pressed"));
      sendLoRaCommand("POWER", "TOGGLE");
      lastPressTimePower = now;
    }
  }

  if (digitalRead(BTN_GUARDIAN) == LOW) {
    if (now - lastPressTimeGuardian > DEBOUNCE_MS) {
      Serial.println(F("BTN: Switch to GUARDIAN Pressed"));
      sendLoRaCommand("SET_MODE", "GUARDIAN");
      lastPressTimeGuardian = now;
    }
  }

  if (digitalRead(BTN_MONITOR) == LOW) {
    if (now - lastPressTimeMonitor > DEBOUNCE_MS) {
      Serial.println(F("BTN: Switch to MONITOR Pressed"));
      sendLoRaCommand("SET_MODE", "MONITOR");
      lastPressTimeMonitor = now;
    }
  }
}

// --- LoRa Command Dispatcher ---
void sendLoRaCommand(const char* action, const char* value) {
  StaticJsonDocument<128> doc;
  doc["cmd"] = action;
  doc["val"] = value;

  char buffer[128];
  size_t len = serializeJson(doc, buffer);

  LoRa.beginPacket();
  LoRa.write((const uint8_t*)buffer, len);
  LoRa.endPacket();

  Serial.print(F("LoRa Tx Command: "));
  Serial.println(buffer);

  lcd.setCursor(14, 0);
  lcd.print("TX");
}

// --- Receive & Parse 30-Second Batch Telemetry from ESP32 ---
void parseIncomingLoRa() {
  int packetSize = LoRa.parsePacket();
  if (packetSize > 0) {
    String payload = "";
    payload.reserve(packetSize);

    while (LoRa.available()) {
      payload += (char)LoRa.read();
    }

    payload.trim();

    if (payload.length() == 0 || payload.charAt(0) != '{') {
      return; 
    }

    bool parsed = false;
    DeserializationError err;

    {
      StaticJsonDocument<256> doc;
      err = deserializeJson(doc, payload);
      if (!err) {
        baseState.temp        = doc["temp"] | 0.0f;
        baseState.hum         = doc["hum"] | 0.0f;
        baseState.voc         = doc["voc"] | 0;
        baseState.sampleCount = doc["samples"] | 0;

        const char* modeStr = doc["mode"] | "UNKNOWN";
        strncpy(baseState.mode, modeStr, sizeof(baseState.mode) - 1);
        baseState.mode[sizeof(baseState.mode) - 1] = '\0';

        baseState.breach = doc["breach"] | false;
        parsed = true;
      }
    }

    if (parsed) {
      // Calculate receiver moving averages
      if (!baseState.lastDataValid) {
        baseState.emaTemp = baseState.temp;
        baseState.emaHum  = baseState.hum;
        baseState.emaVoc  = (float)baseState.voc;
      } else {
        baseState.emaTemp = (EMA_ALPHA * baseState.temp) + ((1.0f - EMA_ALPHA) * baseState.emaTemp);
        baseState.emaHum  = (EMA_ALPHA * baseState.hum)  + ((1.0f - EMA_ALPHA) * baseState.emaHum);
        baseState.emaVoc  = (EMA_ALPHA * (float)baseState.voc) + ((1.0f - EMA_ALPHA) * baseState.emaVoc);
      }

      baseState.lastDataValid = true;
      baseState.lastRxTime    = millis();
      baseState.rssi          = LoRa.packetRssi();
      baseState.snr           = LoRa.packetSnr();
      baseState.packets++;

      // Forward combined metrics to dashboard over USB Serial
      StaticJsonDocument<224> out;
      out["temp"]     = baseState.temp;
      out["hum"]      = baseState.hum;
      out["voc"]      = baseState.voc;
      out["emaTemp"]  = baseState.emaTemp;
      out["emaHum"]   = baseState.emaHum;
      out["samples"]  = baseState.sampleCount;
      out["mode"]     = baseState.mode;
      out["breach"]   = baseState.breach;
      out["rssi"]     = baseState.rssi;
      out["snr"]      = baseState.snr;
      out["packets"]  = baseState.packets;
      
      serializeJson(out, Serial);
      Serial.println();
    } else {
      Serial.print(F("Deserialization failed: "));
      Serial.println(err.c_str());
    }
  }
}

// --- Non-Blocking LCD Render Loop ---
void updateDisplay() {
  static unsigned long lastLcdUpdate = 0;
  if (millis() - lastLcdUpdate < 500) return;
  lastLcdUpdate = millis();

  // Handle offline link condition (e.g. ESP32 Deep Sleep duration exceeds 35s window)
  bool linkOffline = (millis() - baseState.lastRxTime > 40000) || !baseState.lastDataValid;

  if (linkOffline) {
    lcd.setCursor(0, 0);
    lcd.print("LINK: SLEEP/OFF ");
    lcd.setCursor(0, 1);
    lcd.print("Waiting Node... ");
    return;
  }

  char line0[17];
  char shortMode[5] = "INIT";

  if (strncmp(baseState.mode, "GUARDIAN", 8) == 0 || strncmp(baseState.mode, "OVERRIDE", 8) == 0) {
    strncpy(shortMode, "GARD", 5);
  } else if (strncmp(baseState.mode, "MONITOR", 7) == 0 || strncmp(baseState.mode, "INSPECT", 7) == 0) {
    strncpy(shortMode, "INSP", 5);
  }

  // Row 0: Operating Mode, Smoothed Temp & Humidity
  snprintf(line0, sizeof(line0), "%-4s T:%2dC H:%2d%%",
           shortMode,
           (int)baseState.emaTemp,
           (int)baseState.emaHum);

  lcd.setCursor(0, 0);
  lcd.print(line0);

  // Row 1: VOC metric, breach state, and sample count
  char line1[17];
  snprintf(line1, sizeof(line1), "V:%-4d N:%-2u %-5s",
           (int)baseState.emaVoc,
           baseState.sampleCount,
           baseState.breach ? "ALERT" : "OK   ");

  lcd.setCursor(0, 1);
  lcd.print(line1);
}