#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include <semphr.h>

// --- Pin Configurations ---
#define LORA_CS     10
#define LORA_RST    9
#define LORA_DIO0   2

#define BTN_POWER   4   // Single multifunction button on Pin 4

#define LORA_FREQ         868E6
#define DEBOUNCE_MS       50
#define LONG_PRESS_MS     1500

// Expanded Command Engine
enum CommandType : uint8_t {
  CMD_GUARDIAN,
  CMD_MONITOR,
  CMD_TOGGLE_MODE,
  CMD_SLEEP,
  CMD_GSM_CLEAR_SUBS,
  CMD_GSM_ADD_SUB,
  CMD_GSM_SEND_SMS
};

struct CommandMessage {
  CommandType type;
  char payload[32]; // Accommodates phone numbers or quick text payloads
};

struct BaseStationState {
  float temp = 0.0;
  float hum = 0.0;
  int voc = 0;
  uint16_t samples = 0;
  bool breach = false;
  uint8_t gsmSubs = 0;
  char mode[12] = "MONITOR";

  float emaTemp = 0.0;
  float emaHum = 0.0;
  float emaVoc = 0.0;
  bool lastDataValid = false;
  unsigned long lastRxTime = 0;
  unsigned int packets = 0;
};

LiquidCrystal_I2C lcd(0x27, 16, 2);
BaseStationState baseState;

QueueHandle_t xTxQueue = NULL;
SemaphoreHandle_t xStateMutex = NULL;

static void queueCommand(CommandType type, const char* param = "") {
  CommandMessage msg;
  msg.type = type;
  strncpy(msg.payload, param, sizeof(msg.payload) - 1);
  msg.payload[sizeof(msg.payload) - 1] = '\0';
  xQueueSendToBack(xTxQueue, &msg, 0);
}

// Memory diagnostic tool
extern unsigned int __heap_start;
extern void *__brkval;
static int freeRam() {
  int marker;
  return (int)&marker - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}

// --- Dynamic Serial Command Parser ---
// Format supported:
//  - Simple: G, M, T, P, C
//  - Parametric: ADD:+254712345678 or SMS:Alert Test
static void handleSerialCommands() {
  while (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() == 0) continue;

    if (input.equals("G")) {
      queueCommand(CMD_GUARDIAN);
    } else if (input.equals("M")) {
      queueCommand(CMD_MONITOR);
    } else if (input.equals("T")) {
      queueCommand(CMD_TOGGLE_MODE);
    } else if (input.equals("P")) {
      queueCommand(CMD_SLEEP);
    } else if (input.equals("C")) {
      queueCommand(CMD_GSM_CLEAR_SUBS);
    } else if (input.startsWith("ADD:")) {
      String phone = input.substring(4);
      queueCommand(CMD_GSM_ADD_SUB, phone.c_str());
    } else if (input.startsWith("SMS:")) {
      String text = input.substring(4);
      queueCommand(CMD_GSM_SEND_SMS, text.c_str());
    } else {
      Serial.print(F("[CMD ERR] Unknown: "));
      Serial.println(input);
      continue;
    }
    Serial.print(F("[CMD SENT] "));
    Serial.println(input);
  }
}

void vTaskButtons(void *pvParameters);
void vTaskLoRa(void *pvParameters);
void vTaskDisplay(void *pvParameters);

void setup() {
  Serial.begin(115200);
  pinMode(BTN_POWER, INPUT_PULLUP);

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Ground Station"));

  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ)) {
    lcd.setCursor(0, 1);
    lcd.print(F("LoRa FAIL!"));
    while (1);
  }

  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.enableCrc();
  LoRa.receive();

  xTxQueue = xQueueCreate(6, sizeof(CommandMessage));
  xStateMutex = xSemaphoreCreateMutex();

  xTaskCreate(vTaskButtons, "Btn",  128, NULL, 3, NULL);
  xTaskCreate(vTaskLoRa,    "LoRa", 190, NULL, 2, NULL);
  xTaskCreate(vTaskDisplay, "Disp", 160, NULL, 1, NULL);

  Serial.print(F("Free SRAM after tasks: "));
  Serial.println(freeRam());

  vTaskStartScheduler();
}

void loop() {}

// --- Button Task ---
void vTaskButtons(void *pvParameters) {
  bool buttonWasPressed = false;
  unsigned long pressStartTime = 0;

  for (;;) {
    bool isPressed = (digitalRead(BTN_POWER) == LOW);
    unsigned long now = millis();

    if (isPressed && !buttonWasPressed) {
      buttonWasPressed = true;
      pressStartTime = now;
    } 
    else if (!isPressed && buttonWasPressed) {
      buttonWasPressed = false;
      unsigned long duration = now - pressStartTime;

      if (duration >= DEBOUNCE_MS) {
        if (duration >= LONG_PRESS_MS) {
          queueCommand(CMD_SLEEP);
        } else {
          queueCommand(CMD_TOGGLE_MODE);
        }
      }
    }

    handleSerialCommands();
    vTaskDelay(pdMS_TO_TICKS(35));
  }
}

// --- LoRa Communications Engine ---
void vTaskLoRa(void *pvParameters) {
  CommandMessage txMsg;
  static char rxBuffer[160];

  for (;;) {
    // 1. Process & Format Outbound Control JSON Frames
    if (xQueueReceive(xTxQueue, &txMsg, 0) == pdTRUE) {
      LoRa.idle();
      LoRa.beginPacket();
      switch (txMsg.type) {
        case CMD_GUARDIAN:
          LoRa.print(F("{\"cmd\":\"SET_MODE\",\"val\":\"GUARDIAN\"}"));
          break;
        case CMD_MONITOR:
          LoRa.print(F("{\"cmd\":\"SET_MODE\",\"val\":\"MONITOR\"}"));
          break;
        case CMD_TOGGLE_MODE:
          LoRa.print(F("{\"cmd\":\"SET_MODE\",\"val\":\"TOGGLE\"}"));
          break;
        case CMD_SLEEP:
          LoRa.print(F("{\"cmd\":\"POWER\",\"val\":\"OFF\"}"));
          break;
        case CMD_GSM_CLEAR_SUBS:
          LoRa.print(F("{\"cmd\":\"GSM_CLEAR_SUBS\",\"val\":\"\"}"));
          break;
        case CMD_GSM_ADD_SUB:
          LoRa.print(F("{\"cmd\":\"GSM_ADD_SUB\",\"val\":\""));
          LoRa.print(txMsg.payload);
          LoRa.print(F("\"}"));
          break;
        case CMD_GSM_SEND_SMS:
          LoRa.print(F("{\"cmd\":\"GSM_SEND_SMS\",\"val\":\""));
          LoRa.print(txMsg.payload);
          LoRa.print(F("\"}"));
          break;
      }
      LoRa.endPacket();
      vTaskDelay(pdMS_TO_TICKS(10));
      LoRa.receive();
    }

    // 2. Read Inbound LoRa Packets
    int packetSize = LoRa.parsePacket();
    if (packetSize > 0) {
      int bytesRead = 0;
      while (LoRa.available() && bytesRead < (sizeof(rxBuffer) - 1)) {
        rxBuffer[bytesRead++] = (char)LoRa.read();
      }
      rxBuffer[bytesRead] = '\0';

      baseState.packets++;

      // Serial Passthrough for PC Dashboard
      if (bytesRead > 1 && rxBuffer[bytesRead - 1] == '}') {
        Serial.write((const uint8_t *)rxBuffer, bytesRead - 1);
        Serial.print(F(",\"rssi\":"));    Serial.print(LoRa.packetRssi());
        Serial.print(F(",\"snr\":"));     Serial.print(LoRa.packetSnr(), 1);
        Serial.print(F(",\"packets\":")); Serial.print(baseState.packets);
        Serial.println('}');
      }

      // Zero-Heap Field Extraction
      char *tempPtr   = strstr(rxBuffer, "\"temp\":");
      char *humPtr    = strstr(rxBuffer, "\"hum\":");
      char *vocPtr    = strstr(rxBuffer, "\"voc\":");
      char *sampPtr   = strstr(rxBuffer, "\"samples\":");
      char *modePtr   = strstr(rxBuffer, "\"mode\":");
      char *breachPtr = strstr(rxBuffer, "\"breach\":");
      char *gsmSubPtr = strstr(rxBuffer, "\"gsm_subs\":");

      if (tempPtr != NULL && xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
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

        // Apply Exponential Moving Average (EMA)
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
        xSemaphoreGive(xStateMutex);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(40));
  }
}

// --- Display Task ---
void vTaskDisplay(void *pvParameters) {
  for (;;) {
    if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      bool offline = (millis() - baseState.lastRxTime > 40000) || !baseState.lastDataValid;

      if (offline) {
        lcd.setCursor(0, 0);
        lcd.print(F("NODE: OFFLINE   "));
        lcd.setCursor(0, 1);
        lcd.print(F("Waiting Data... "));
      } else {
        // Top Line: Temperature, Humidity, VOC
        lcd.setCursor(0, 0);
        lcd.print(F("T:"));
        lcd.print((int)baseState.emaTemp);
        lcd.print(F("C H:"));
        lcd.print((int)baseState.emaHum);
        lcd.print(F("% V:"));
        lcd.print((int)baseState.emaVoc);
        lcd.print(F("   "));

        // Bottom Line: Mode & GSM Subscriber Count / Breach Flag
        lcd.setCursor(0, 1);
        if (baseState.breach) {
          lcd.print(F("!ALARM BREACH!  "));
        } else {
          lcd.print(F("M:"));
          lcd.print(baseState.mode);
          lcd.print(F(" S:"));
          lcd.print(baseState.gsmSubs);
          lcd.print(F("   "));
        }
      }
      xSemaphoreGive(xStateMutex);
    }
    vTaskDelay(pdMS_TO_TICKS(400));
  }
}