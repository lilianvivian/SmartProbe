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
};

LiquidCrystal_I2C lcd(0x27, 16, 2);
BaseStationState baseState;

QueueHandle_t xTxQueue = NULL;
SemaphoreHandle_t xStateMutex = NULL;

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

  vTaskStartScheduler();
}

void loop() {}

void vTaskButtons(void *pvParameters) {
  TickType_t xLastPower = 0, xLastGuardian = 0, xLastMonitor = 0;

  for (;;) {
    TickType_t xNow = xTaskGetTickCount();

    if (digitalRead(BTN_POWER) == LOW && (xNow - xLastPower > pdMS_TO_TICKS(DEBOUNCE_MS))) {
      xLastPower = xNow;
      CommandMessage msg = {"POWER", "TOGGLE"};
      xQueueSendToBack(xTxQueue, &msg, 0);
    }

    if (digitalRead(BTN_GUARDIAN) == LOW && (xNow - xLastGuardian > pdMS_TO_TICKS(DEBOUNCE_MS))) {
      xLastGuardian = xNow;
      CommandMessage msg = {"SET_MODE", "GUARDIAN"};
      xQueueSendToBack(xTxQueue, &msg, 0);
    }

    if (digitalRead(BTN_MONITOR) == LOW && (xNow - xLastMonitor > pdMS_TO_TICKS(DEBOUNCE_MS))) {
      xLastMonitor = xNow;
      CommandMessage msg = {"SET_MODE", "MONITOR"};
      xQueueSendToBack(xTxQueue, &msg, 0);
    }

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

      Serial.print(F("[RX RAW] "));
      Serial.println(rxBuffer);

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
          baseState.emaTemp = (0.3f * baseState.temp) + (0.7f * baseState.emaHum);
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