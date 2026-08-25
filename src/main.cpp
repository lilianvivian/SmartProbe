#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include <semphr.h>

// --- Pin Definitions ---
#define LORA_CS     10
#define LORA_RST    9
#define LORA_DIO0   2

#define BTN_POWER    3
#define BTN_GUARDIAN 4
#define BTN_MONITOR  5

#define LORA_FREQ    868E6
#define DEBOUNCE_MS  250

struct CommandMessage {
  char cmd[10];
  char val[10];
};

struct BaseStationState {
  float temp = 0.0;
  float hum = 0.0;
  int voc = 0;
  char mode[10] = "MONITOR";
  float emaTemp = 0.0;
  float emaHum = 0.0;
  bool lastDataValid = false;
  unsigned long lastRxTime = 0;
  unsigned int packets = 0;   // received since power-up, reported to the dashboard
};

LiquidCrystal_I2C lcd(0x27, 16, 2);
BaseStationState baseState;

QueueHandle_t xTxQueue = NULL;
SemaphoreHandle_t xStateMutex = NULL;

// Command literals held in flash. A plain string literal on AVR is copied into
// .data at startup and costs SRAM for the whole run; PROGMEM keeps it in
// program space and pays only a transient copy when a command is queued.
const char CMD_SET_MODE[] PROGMEM = "SET_MODE";
const char CMD_POWER[]    PROGMEM = "POWER";
const char VAL_GUARDIAN[] PROGMEM = "GUARDIAN";
const char VAL_MONITOR[]  PROGMEM = "MONITOR";
const char VAL_TOGGLE[]   PROGMEM = "TOGGLE";

// Single entry point for both input sources, so a button press and a dashboard
// click are indistinguishable downstream.
static void queueCommand(const char *cmdP, const char *valP) {
  CommandMessage msg;
  strncpy_P(msg.cmd, cmdP, sizeof(msg.cmd) - 1);
  msg.cmd[sizeof(msg.cmd) - 1] = '\0';
  strncpy_P(msg.val, valP, sizeof(msg.val) - 1);
  msg.val[sizeof(msg.val) - 1] = '\0';
  xQueueSendToBack(xTxQueue, &msg, 0);
}

// The dashboard sends one byte per command, so this needs no line buffer, no
// parser and no JSON library — the whole USB input path costs zero persistent
// SRAM. G = arm, M = disarm, P = sleep.
static void handleSerialCommands() {
  while (Serial.available()) {
    switch (Serial.read()) {
      case 'G': queueCommand(CMD_SET_MODE, VAL_GUARDIAN); break;
      case 'M': queueCommand(CMD_SET_MODE, VAL_MONITOR);  break;
      case 'P': queueCommand(CMD_POWER,    VAL_TOGGLE);   break;
      default:  break;   // newlines and stray bytes are ignored
    }
  }
}

// Measured headroom beats assumed headroom: this is the gap between the top of
// the heap (task stacks included) and the current stack pointer.
extern unsigned int __heap_start;
extern void *__brkval;

static int freeRam() {
  int marker;
  return (int)&marker - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}

void vTaskButtons(void *pvParameters);
void vTaskLoRa(void *pvParameters);
void vTaskDisplay(void *pvParameters);

void setup() {
  Serial.begin(115200);

  pinMode(BTN_POWER, INPUT_PULLUP);
  pinMode(BTN_GUARDIAN, INPUT_PULLUP);
  pinMode(BTN_MONITOR, INPUT_PULLUP);

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
  LoRa.setCodingRate4(5); // Must match ESP32 Sync Word
  LoRa.enableCrc();
  LoRa.receive();

  xTxQueue = xQueueCreate(4, sizeof(CommandMessage));
  xStateMutex = xSemaphoreCreateMutex();

  xTaskCreate(vTaskButtons, "Btn", 100, NULL, 3, NULL);
  xTaskCreate(vTaskLoRa,    "LoRa", 240, NULL, 2, NULL);
  xTaskCreate(vTaskDisplay, "Disp", 160, NULL, 1, NULL);

  // Printed after the stacks are allocated, so this is the real remaining
  // headroom rather than the static figure the linker reports.
  Serial.print(F("Free SRAM after task creation: "));
  Serial.println(freeRam());

  vTaskStartScheduler();
}

void loop() {}

void vTaskButtons(void *pvParameters) {
  TickType_t xLastPower = 0, xLastGuardian = 0, xLastMonitor = 0;

  for (;;) {
    TickType_t xNow = xTaskGetTickCount();

    if (digitalRead(BTN_POWER) == LOW && (xNow - xLastPower > pdMS_TO_TICKS(DEBOUNCE_MS))) {
      xLastPower = xNow;
      queueCommand(CMD_POWER, VAL_TOGGLE);
    }

    if (digitalRead(BTN_GUARDIAN) == LOW && (xNow - xLastGuardian > pdMS_TO_TICKS(DEBOUNCE_MS))) {
      xLastGuardian = xNow;
      queueCommand(CMD_SET_MODE, VAL_GUARDIAN);
    }

    if (digitalRead(BTN_MONITOR) == LOW && (xNow - xLastMonitor > pdMS_TO_TICKS(DEBOUNCE_MS))) {
      xLastMonitor = xNow;
      queueCommand(CMD_SET_MODE, VAL_MONITOR);
    }

    // The dashboard is just a fourth input onto the same queue — no extra task,
    // no extra stack.
    handleSerialCommands();

    vTaskDelay(pdMS_TO_TICKS(35));
  }
}

void vTaskLoRa(void *pvParameters) {
  CommandMessage txMsg;
  char rxBuffer[128];

  for (;;) {
    // 1. Send Queued Commands
    if (xQueueReceive(xTxQueue, &txMsg, 0) == pdTRUE) {
      LoRa.idle();
      LoRa.beginPacket();
      LoRa.print(F("{\"cmd\":\""));
      LoRa.print(txMsg.cmd);
      LoRa.print(F("\",\"val\":\""));
      LoRa.print(txMsg.val);
      LoRa.print(F("\"}"));
      LoRa.endPacket();
      
      vTaskDelay(pdMS_TO_TICKS(10));
      LoRa.receive();
    }

    // 2. Read Packets Safely Into Fixed Static Buffer
    int packetSize = LoRa.parsePacket();
    if (packetSize > 0) {
      int bytesRead = 0;
      while (LoRa.available() && bytesRead < (sizeof(rxBuffer) - 1)) {
        rxBuffer[bytesRead++] = (char)LoRa.read();
      }
      rxBuffer[bytesRead] = '\0'; // Explicit Null-Termination

      baseState.packets++;

      // One line for the dashboard: the node's own payload with radio stats
      // spliced in before the closing brace. Splicing rather than rebuilding
      // preserves fields this board never parses (voc, breach) and needs no
      // output buffer of its own.
      if (bytesRead > 1 && rxBuffer[bytesRead - 1] == '}') {
        Serial.write((const uint8_t *)rxBuffer, bytesRead - 1);
        Serial.print(F(",\"rssi\":"));    Serial.print(LoRa.packetRssi());
        Serial.print(F(",\"snr\":"));     Serial.print(LoRa.packetSnr(), 1);
        Serial.print(F(",\"packets\":")); Serial.print(baseState.packets);
        Serial.println('}');
      } else {
        Serial.print(F("[RX MALFORMED] "));
        Serial.println(rxBuffer);
      }

      // Simple parsing using string search
      char *tempPtr = strstr(rxBuffer, "\"temp\":");
      char *humPtr  = strstr(rxBuffer, "\"hum\":");
      char *modePtr = strstr(rxBuffer, "\"mode\":");

      if (tempPtr != NULL && xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        baseState.temp = atof(tempPtr + 7);
        if (humPtr != NULL) {
          baseState.hum = atof(humPtr + 6);
        }

        if (modePtr != NULL) {
          char *modeStart = strchr(modePtr + 7, '"');
          if (modeStart != NULL) {
            modeStart++;
            char *modeEnd = strchr(modeStart, '"');
            if (modeEnd != NULL) {
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
        } else {
          baseState.emaTemp = (0.3f * baseState.temp) + (0.7f * baseState.emaTemp);
          baseState.emaHum  = (0.3f * baseState.hum)  + (0.7f * baseState.emaHum);
        }

        baseState.lastDataValid = true;
        baseState.lastRxTime = millis();
        xSemaphoreGive(xStateMutex);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(40));
  }
}

void vTaskDisplay(void *pvParameters) {
  for (;;) {
    if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      bool offline = (millis() - baseState.lastRxTime > 40000) || !baseState.lastDataValid;

      lcd.setCursor(0, 0);
      if (offline) {
        lcd.print(F("LINK: SLEEP/OFF "));
        lcd.setCursor(0, 1);
        lcd.print(F("Waiting Node... "));
      } else {
        if (baseState.mode[0] == 'G' || baseState.mode[0] == 'O') {
          lcd.print(F("GARD "));
        } else {
          lcd.print(F("INSP "));
        }

        lcd.print(F("T:"));
        lcd.print((int)baseState.emaTemp);
        lcd.print(F("C H:"));
        lcd.print((int)baseState.emaHum);
        lcd.print(F("%   "));

        lcd.setCursor(0, 1);
        lcd.print(F("Status: Active  "));
      }
      xSemaphoreGive(xStateMutex);
    }
    vTaskDelay(pdMS_TO_TICKS(400));
  }
}