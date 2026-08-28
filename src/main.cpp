#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define LORA_CS          10
#define LORA_RST         9
#define LORA_DIO0        2
#define BTN_POWER        7  

#define LORA_FREQ        868E6
#define DEBOUNCE_MS      50
#define DISPLAY_INTERVAL 400

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

bool buttonWasPressed = false;
unsigned long pressStartTime = 0;
unsigned long lastDisplayUpdate = 0;

// Hardware Error Guarded TX Function
bool sendLoRaCommand(const char* payload) {
  if (payload == NULL || strlen(payload) == 0) {
    Serial.println(F("[ERROR] Attempted to transmit NULL/Empty payload"));
    return false;
  }

  Serial.print(F("[LORA TX] "));
  Serial.println(payload);

  LoRa.idle();
  if (!LoRa.beginPacket()) {
    Serial.println(F("[ERROR] Failed to start LoRa packet transmission"));
    return false;
  }
  
  LoRa.print(payload);
  
  if (!LoRa.endPacket()) {
    Serial.println(F("[ERROR] LoRa Packet Transmission Failed"));
    return false;
  }
  
  delay(10);
  LoRa.receive();
  return true;
}

void handleButton() {
  bool isPressed = (digitalRead(BTN_POWER) == LOW);
  unsigned long now = millis();

  if (isPressed && !buttonWasPressed) {
    buttonWasPressed = true;
    pressStartTime = now;
  } 
  else if (!isPressed && buttonWasPressed) {
    buttonWasPressed = false;
    if ((now - pressStartTime) >= DEBOUNCE_MS) {
      sendLoRaCommand("{\"cmd\":\"POWER\",\"val\":\"OFF\"}");
    }
  }
}

void handleSerialCommands() {
  if (!Serial.available()) return;
  
  char buf[32];
  // Safe buffer bounded read
  uint8_t len = Serial.readBytesUntil('\n', buf, sizeof(buf) - 1);
  buf[len] = '\0';
  
  while (len > 0 && (buf[len - 1] == '\r' || buf[len - 1] == ' ')) {
    buf[--len] = '\0';
  }
  if (len == 0) return;

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
    int written = snprintf(cmdBuf, sizeof(cmdBuf), "{\"cmd\":\"GSM_ADD_SUB\",\"val\":\"%s\"}", buf + 4);
    if (written > 0 && written < (int)sizeof(cmdBuf)) {
      sendLoRaCommand(cmdBuf);
    } else {
      Serial.println(F("[ERROR] ADD: Buffer overflow prevented"));
    }
  } else if (strncmp(buf, "SMS:", 4) == 0) {
    char cmdBuf[48];
    int written = snprintf(cmdBuf, sizeof(cmdBuf), "{\"cmd\":\"GSM_SEND_SMS\",\"val\":\"%s\"}", buf + 4);
    if (written > 0 && written < (int)sizeof(cmdBuf)) {
      sendLoRaCommand(cmdBuf);
    } else {
      Serial.println(F("[ERROR] SMS: Buffer overflow prevented"));
    }
  }
}

void handleLoRaRx() {
  static char rxBuffer[128];
  int packetSize = LoRa.parsePacket();

  if (packetSize <= 0) return;

  int bytesRead = 0;
  while (LoRa.available() && bytesRead < (sizeof(rxBuffer) - 1)) {
    rxBuffer[bytesRead++] = (char)LoRa.read();
  }
  rxBuffer[bytesRead] = '\0';

  // Guard against truncated or zero-length packets
  if (bytesRead == 0) return;

  baseState.packets++;

  // Output to Serial Web Dashboard pipeline safely
  if (bytesRead > 1 && rxBuffer[bytesRead - 1] == '}') {
    Serial.write((const uint8_t *)rxBuffer, bytesRead - 1);
    Serial.print(F(",\"rssi\":"));    Serial.print(LoRa.packetRssi());
    Serial.print(F(",\"snr\":"));     Serial.print(LoRa.packetSnr(), 1);
    Serial.print(F(",\"packets\":")); Serial.print(baseState.packets);
    Serial.println('}');
  }

  // Safe Pointers with Defensive Bounds Checks
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

    if (modePtr != NULL) {
      char *modeStart = strchr(modePtr + 7, '"');
      if (modeStart != NULL) {
        modeStart++;
        char *modeEnd = strchr(modeStart, '"');
        if (modeEnd != NULL && modeEnd > modeStart) {
          size_t len = modeEnd - modeStart;
          if (len < sizeof(baseState.mode)) {
            strncpy(baseState.mode, modeStart, len);
            baseState.mode[len] = '\0';
          } else {
            // Memory guard against long mode strings
            strncpy(baseState.mode, modeStart, sizeof(baseState.mode) - 1);
            baseState.mode[sizeof(baseState.mode) - 1] = '\0';
          }
        }
      }
    }

    // Exponential Moving Average (EMA) Calculation
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
    lcd.setCursor(0, 0);
    lcd.print(F("T:"));
    lcd.print(baseState.emaTemp, 1);
    lcd.print(F("C H:"));
    lcd.print((int)baseState.emaHum);
    lcd.print(F("%    "));

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
  pinMode(BTN_POWER, INPUT_PULLUP);

  // Initialize display with feedback
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Ground Station"));

  // Hardware Reset LoRa Transceiver
  pinMode(LORA_RST, OUTPUT);
  digitalWrite(LORA_RST, LOW);
  delay(10);
  digitalWrite(LORA_RST, HIGH);
  delay(10);

  // Error Trap: LoRa Hardware Initialization Check
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println(F("[FATAL] LoRa Initialization Failed!"));
    lcd.setCursor(0, 1);
    lcd.print(F("LoRa FAIL!"));
    
    // Halt execution gracefully rather than allowing undefined behavior
    while (1) {
      delay(1000);
    }
  }

  // Radio Configuration
  LoRa.setTxPower(17);
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setPreambleLength(8);
  LoRa.setSyncWord(0x12);
  LoRa.enableCrc();
  LoRa.receive();

  Serial.println(F("[SYSTEM] Ground Station Ready. Ready for WebSerial connection."));
}

void loop() {
  handleButton();
  handleSerialCommands();
  handleLoRaRx();
  updateDisplay();
}