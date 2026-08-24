#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ArduinoJson.h>

// --- Pin Definitions ---
#define LORA_CS    10
#define LORA_RST   9
#define LORA_DIO0  2

#define BTN_POWER    3
#define BTN_GUARDIAN 4
#define BTN_MONITOR  5

#define LORA_FREQ    868E6  // Set to match regional frequency (433E6, 868E6, 915E6)
#define DEBOUNCE_MS  200

// --- Hardware Objects ---
LiquidCrystal_I2C lcd(0x27, 16, 2); 

// --- System Telemetry State Structure ---
struct BaseStationState {
  float temp = 0.0;
  float hum = 0.0;
  int voc = 0;
  char mode[10] = "INIT";
  bool breach = false;
  bool lastDataValid = false;
  unsigned long lastRxTime = 0;
  int rssi = 0;             // dBm of the last packet
  float snr = 0.0;          // dB
  unsigned int packets = 0; // received since power-up
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

  // Radio Configurations (Must match ESP32 node exactly)
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
// The dashboard drives the same commands as the three physical buttons, sent as
// newline-terminated JSON over USB: {"cmd":"SET_MODE","val":"GUARDIAN"}
// Everything funnels into sendLoRaCommand(), so the node cannot tell whether a
// command came from a button or the browser.
void handleSerialCommands() {
  static char buf[64];
  static byte len = 0;

  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (len == 0) continue;          // ignore bare line endings
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
      len = 0;   // overlong line: discard rather than truncate into a bad parse
    }
  }
}

// --- Button Handler Routine ---
void handleButtons() {
  unsigned long now = millis();

  // Button 1: Power Toggle Command
  if (digitalRead(BTN_POWER) == LOW) {
    if (now - lastPressTimePower > DEBOUNCE_MS) {
      Serial.println(F("BTN: Power Toggle Pressed"));
      sendLoRaCommand("POWER", "TOGGLE");
      lastPressTimePower = now;
    }
  }

  // Button 2: Force Guardian Mode Command
  if (digitalRead(BTN_GUARDIAN) == LOW) {
    if (now - lastPressTimeGuardian > DEBOUNCE_MS) {
      Serial.println(F("BTN: Switch to GUARDIAN Pressed"));
      sendLoRaCommand("SET_MODE", "GUARDIAN");
      lastPressTimeGuardian = now;
    }
  }

  // Button 3: Force Monitor/Inspect Mode Command
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
  // Use explicit static capacity for AVR safety
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
// --- Receive & Parse Base Station Telemetry ---
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

    // Static buffer prevents SRAM heap fragmentation on ATmega328P.
    //
    // The parse document is deliberately confined to its own scope: every field
    // is copied into baseState below, so holding it open while the 192-byte
    // output document is built would put 448 bytes of JsonDocument on the stack
    // at once. With ~1 KB of free SRAM that is close enough to the edge to
    // corrupt the heap, and the LoRa driver's state sits in it.
    bool parsed = false;
    DeserializationError err;   // 1-byte value type, safe to outlive the doc
    {
      StaticJsonDocument<256> doc;
      err = deserializeJson(doc, payload);
      if (!err) {
        baseState.temp = doc["temp"] | 0.0f;
        baseState.hum = doc["hum"] | 0.0f;
        baseState.voc = doc["voc"] | 0;

        const char* modeStr = doc["mode"] | "UNKNOWN";
        strncpy(baseState.mode, modeStr, sizeof(baseState.mode) - 1);
        baseState.mode[sizeof(baseState.mode) - 1] = '\0';

        baseState.breach = doc["breach"] | false;
        parsed = true;
      }
    }   // doc released here - its 256 bytes are reclaimed before `out` exists

    if (parsed) {
      baseState.lastDataValid = true;
      baseState.lastRxTime = millis();
      baseState.rssi = LoRa.packetRssi();
      baseState.snr  = LoRa.packetSnr();
      baseState.packets++;

      // Re-emit for the USB dashboard with the radio stats folded in. Built
      // with ArduinoJson rather than snprintf because AVR's printf has no
      // float support - "%f" prints nothing on an ATmega328P.
      StaticJsonDocument<192> out;
      out["temp"]    = baseState.temp;
      out["hum"]     = baseState.hum;
      out["voc"]     = baseState.voc;
      out["mode"]    = baseState.mode;
      out["breach"]  = baseState.breach;
      out["rssi"]    = baseState.rssi;
      out["snr"]     = baseState.snr;
      out["packets"] = baseState.packets;
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
  if (millis() - lastLcdUpdate < 500) return; // Refresh LCD 2x per second
  lastLcdUpdate = millis();

  // Check connection timeout (10 second threshold)
  bool linkOffline = (millis() - baseState.lastRxTime > 10000) || !baseState.lastDataValid;

  if (linkOffline) {
    lcd.setCursor(0, 0);
    lcd.print("LINK: OFFLINE   ");
    lcd.setCursor(0, 1);
    lcd.print("Waiting Base... ");
    return;
  }

  // Line 1: Mode, Temperature, and Humidity
  char line0[17];
  char shortMode[5] = "INIT";

  if (strncmp(baseState.mode, "GUARDIAN", 8) == 0 || strncmp(baseState.mode, "OVERRIDE", 8) == 0) {
    strncpy(shortMode, "GARD", 5);
  } else if (strncmp(baseState.mode, "MONITOR", 7) == 0 || strncmp(baseState.mode, "INSPECT", 7) == 0) {
    strncpy(shortMode, "INSP", 5);
  }

  snprintf(line0, sizeof(line0), "%-4s T:%2dC H:%2d%%",
           shortMode,
           (int)baseState.temp,
           (int)baseState.hum);

  lcd.setCursor(0, 0);
  lcd.print(line0);

  // Line 2: VOC Air Quality Level & Alarm Status
  char line1[17];
  snprintf(line1, sizeof(line1), "VOC:%-4d ST:%-5s",
           baseState.voc,
           baseState.breach ? "ALERT" : "OK   ");

  lcd.setCursor(0, 1);
  lcd.print(line1);
}