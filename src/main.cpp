#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// --- Pin Configurations ---
#define LORA_CS     10
#define LORA_RST    9
#define LORA_DIO0   2
#define BTN_POWER   7  // Wire switch between Pin 4 and GND

#define LORA_FREQ         868E6
#define DEBOUNCE_MS       50
#define DISPLAY_INTERVAL  400

struct BaseStationState {
  float temp = 0.0;
  float hum = 0.0;
  int voc = 0;
  uint16_t samples = 0;
  bool breach = false;
  uint8_t gsmSubs = 0;
  char mode[10] = "MONITOR";

  float emaTemp = 0.0;
  float emaHum = 0.0;
  float emaVoc = 0.0;
  bool lastDataValid = false;
  unsigned long lastRxTime = 0;
  unsigned int packets = 0;
};

LiquidCrystal_I2C lcd(0x27, 16, 2);
BaseStationState baseState;

// Variable tracking for button state machine
bool buttonWasPressed = false;
unsigned long pressStartTime = 0;
unsigned long lastDisplayUpdate = 0;

// Direct LoRa Transmission Helper
void sendLoRaCommand(const char* payload) {
  Serial.print(F("[LORA TX] Transmitting: "));
  Serial.println(payload);

  LoRa.idle();
  LoRa.beginPacket();
  LoRa.print(payload);
  LoRa.endPacket();
  
  delay(10);
  LoRa.receive();
  Serial.println(F("[LORA TX] Complete. Radio in RX mode."));
}

void handleButton() {
  bool isPressed = (digitalRead(BTN_POWER) == LOW);
  unsigned long now = millis();

  if (isPressed && !buttonWasPressed) {
    buttonWasPressed = true;
    pressStartTime = now;
    Serial.println(F("[BTN] Pin 4 Pressed (LOW detected)"));
  } 
  else if (!isPressed && buttonWasPressed) {
    buttonWasPressed = false;
    unsigned long duration = now - pressStartTime;

    Serial.print(F("[BTN] Pin 4 Released. Hold duration: "));
    Serial.print(duration);
    Serial.println(F(" ms"));

    if (duration >= DEBOUNCE_MS) {
      Serial.println(F("[BTN LOGIC] Valid Press. Sending POWER OFF..."));
      sendLoRaCommand("{\"cmd\":\"POWER\",\"val\":\"OFF\"}");
    } else {
      Serial.println(F("[BTN LOGIC] Ignored as noise/bounce (<50ms)."));
    }
  }
}

void handleSerialCommands() {
  if (!Serial.available()) return;
  
  char buf[32];
  uint8_t len = Serial.readBytesUntil('\n', buf, sizeof(buf) - 1);
  buf[len] = '\0';
  
  while (len > 0 && (buf[len - 1] == '\r' || buf[len - 1] == ' ')) {
    buf[--len] = '\0';
  }
  if (len == 0) return;

  Serial.print(F("[SERIAL IN] Command received: "));
  Serial.println(buf);

  if (strcmp(buf, "G") == 0) {
    sendLoRaCommand("{\"cmd\":\"SET_MODE\",\"val\":\"GUARDIAN\"}");
  } else if (strcmp(buf, "M") == 0) {
    sendLoRaCommand("{\"cmd\":\"SET_MODE\",\"val\":\"MONITOR\"}");
  } else if (strcmp(buf, "T") == 0) {
    sendLoRaCommand("{\"cmd\":\"SET_MODE\",\"val\":\"TOGGLE\"}");
  } else if (strcmp(buf, "P") == 0) {
    sendLoRaCommand("{\"cmd\":\"POWER\",\"val\":\"OFF\"}");
  } else if (strcmp(buf, "C") == 0) {
    sendLoRaCommand("{\"cmd\":\"GSM_CLEAR_SUBS\",\"val\":\"\"}");
  } else if (strncmp(buf, "ADD:", 4) == 0) {
    char cmdBuf[48];
    snprintf(cmdBuf, sizeof(cmdBuf), "{\"cmd\":\"GSM_ADD_SUB\",\"val\":\"%s\"}", buf + 4);
    sendLoRaCommand(cmdBuf);
  } else if (strncmp(buf, "SMS:", 4) == 0) {
    char cmdBuf[48];
    snprintf(cmdBuf, sizeof(cmdBuf), "{\"cmd\":\"GSM_SEND_SMS\",\"val\":\"%s\"}", buf + 4);
    sendLoRaCommand(cmdBuf);
  } else {
    Serial.print(F("[CMD ERR] Unknown command format: "));
    Serial.println(buf);
  }
}

void handleLoRaRx() {
  static char rxBuffer[100];
  int packetSize = LoRa.parsePacket();

  if (packetSize > 0) {
    Serial.print(F("\n[LORA RX] Packet received! Size: "));
    Serial.print(packetSize);
    Serial.println(F(" bytes."));

    int bytesRead = 0;
    while (LoRa.available() && bytesRead < (sizeof(rxBuffer) - 1)) {
      rxBuffer[bytesRead++] = (char)LoRa.read();
    }
    rxBuffer[bytesRead] = '\0';

    baseState.packets++;

    // Print JSON output for Web Dashboard
    if (bytesRead > 1 && rxBuffer[bytesRead - 1] == '}') {
      Serial.write((const uint8_t *)rxBuffer, bytesRead - 1);
      Serial.print(F(",\"rssi\":"));    Serial.print(LoRa.packetRssi());
      Serial.print(F(",\"snr\":"));     Serial.print(LoRa.packetSnr(), 1);
      Serial.print(F(",\"packets\":")); Serial.print(baseState.packets);
      Serial.println('}');
    } else {
      Serial.print(F("[LORA RX RAW] "));
      Serial.println(rxBuffer);
    }

    char *tempPtr   = strstr(rxBuffer, "\"temp\":");
    char *humPtr    = strstr(rxBuffer, "\"hum\":");
    char *vocPtr    = strstr(rxBuffer, "\"voc\":");
    char *sampPtr   = strstr(rxBuffer, "\"samples\":");
    char *modePtr   = strstr(rxBuffer, "\"mode\":");
    char *breachPtr = strstr(rxBuffer, "\"breach\":");
    char *gsmSubPtr = strstr(rxBuffer, "\"gsm_subs\":");

    if (tempPtr != NULL) {
      baseState.temp = atof(tempPtr + 7);
      if (humPtr)    baseState.hum     = atof(humPtr + 6);
      if (vocPtr)    baseState.voc     = atoi(vocPtr + 6);
      if (sampPtr)   baseState.samples = atoi(sampPtr + 10);
      if (breachPtr) baseState.breach  = (strstr(breachPtr + 9, "true") != NULL);
      if (gsmSubPtr) baseState.gsmSubs = atoi(gsmSubPtr + 11);

      if (modePtr) {
        char *modeStart = strchr(modePtr + 7, '"');
        if (modeStart) {
          modeStart++;
          char *modeEnd = strchr(modeStart, '"');
          if (modeEnd) {
            size_t len = modeEnd - modeStart;
            if (len < sizeof(baseState.mode)) {
              strncpy(baseState.mode, modeStart, len);
              baseState.mode[len] = '\0';
            }
          }
        }
      }

      if (!baseState.lastDataValid) {
        baseState.emaTemp = baseState.temp;
        baseState.emaHum  = baseState.hum;
        baseState.emaVoc  = (float)baseState.voc;
      } else {
        baseState.emaTemp = (0.3f * baseState.temp) + (0.7f * baseState.emaTemp);
        baseState.emaHum  = (0.3f * baseState.hum)  + (0.7f * baseState.emaHum);
        baseState.emaVoc  = (0.3f * (float)baseState.voc) + (0.7f * baseState.emaVoc);
      }

      baseState.lastDataValid = true;
      baseState.lastRxTime = millis();

      Serial.print(F("[PARSER] Temp: ")); Serial.print(baseState.temp);
      Serial.print(F("C | Hum: "));       Serial.print(baseState.hum);
      Serial.print(F("% | VOC: "));       Serial.println(baseState.voc);
    }
  }
}

void updateDisplay() {
  if (millis() - lastDisplayUpdate < DISPLAY_INTERVAL) return;
  lastDisplayUpdate = millis();

  bool offline = (millis() - baseState.lastRxTime > 40000) || !baseState.lastDataValid;

  if (offline) {
    lcd.setCursor(0, 0);
    lcd.print(F("NODE: OFFLINE   "));
    lcd.setCursor(0, 1);
    lcd.print(F("Waiting Data... "));
  } else {
    // Line 1: Temp & Humidity
    lcd.setCursor(0, 0);
    lcd.print(F("T:"));
    lcd.print(baseState.emaTemp, 1);
    lcd.print(F("C H:"));
    lcd.print((int)baseState.emaHum);
    lcd.print(F("%   "));

    // Line 2: VOC & System State
    lcd.setCursor(0, 1);
    if (baseState.breach) {
      lcd.print(F("V:"));
      lcd.print((int)baseState.emaVoc);
      lcd.print(F(" !ALARM!   "));
    } else {
      lcd.print(F("V:"));
      lcd.print((int)baseState.emaVoc);
      lcd.print(F(" "));
      lcd.print(baseState.mode);
      lcd.print(F("    "));
    }
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  Serial.println(F("\n======================================"));
  Serial.println(F("[SYS INIT] Ground Station (Single Loop)"));
  Serial.println(F("======================================"));

  pinMode(BTN_POWER, INPUT_PULLUP);

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Ground Station"));

  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println(F("[LORA ERR] Init Failed!"));
    lcd.setCursor(0, 1);
    lcd.print(F("LoRa FAIL!"));
    while (1);
  }

  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.enableCrc();
  LoRa.receive();
  Serial.println(F("[SYS] Hardware initialization complete. Entering main loop..."));
}

void loop() {
  handleButton();
  handleSerialCommands();
  handleLoRaRx();
  updateDisplay();
}