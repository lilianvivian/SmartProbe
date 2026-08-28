#include <Arduino.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <SPI.h>
#include <LoRa.h>
#include <DHT.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/event_groups.h>
#include "pins.h"
#include <nvs_flash.h>
#include <driver/rtc_io.h>

#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "grain_model.h"

#ifndef g_model_data
#ifdef g_grain_model
#define g_model_data g_grain_model
#endif
#endif

#define PUMP_PWM_CHANNEL     0
#define FAN_PWM_CHANNEL      1
#define PWM_FREQ             1000
#define PWM_RES              8
#define LORA_FREQ            868E6

#define MODE_INSPECT_BIT     (1UL << 0UL)
#define MODE_OVERRIDE_BIT    (1UL << 1UL)
#define THRESHOLD_BREACH_BIT (1UL << 2UL)
#define ALL_MODES_MASK       (MODE_INSPECT_BIT | MODE_OVERRIDE_BIT)

#define AWAKE_INTERVAL_SEC   (2 * 60)
#define TX_FAST_INTERVAL_MS  5000 
#define EMA_ALPHA            0.2f
#define MAX_SUBSCRIBERS      5
#define PHONE_LEN            16

#define BUTTON_WAKEUP_MASK   (1ULL << POWER_BUTTON)

RTC_DATA_ATTR bool sysPowerState = true;

namespace {
  tflite::ErrorReporter* errorReporter = nullptr;
  const tflite::Model* model = nullptr;
  tflite::MicroInterpreter* interpreter = nullptr;
  TfLiteTensor* input = nullptr;
  TfLiteTensor* output = nullptr;
  constexpr int kTensorArenaSize = 8 * 1024;
  uint8_t tensorArena[kTensorArenaSize];
  bool isTinyMlReady = false;
}

enum SystemCommandType {
  CMD_SET_MODE_INSPECT,
  CMD_SET_MODE_OVERRIDE,
  CMD_TOGGLE_MODE,
  CMD_TRIGGER_SLEEP
};

DHT dht(DHT_PIN, DHT_TYPE);
HardwareSerial gsmSerial(2);
Preferences preferences;

EventGroupHandle_t xSystemEvents = NULL;
QueueHandle_t gsmQueue = NULL;
QueueHandle_t systemCmdQueue = NULL;
QueueHandle_t loraTxQueue = NULL;
SemaphoreHandle_t xPowerSleepSemaphore = NULL;
SemaphoreHandle_t xActivityMutex = NULL;
SemaphoreHandle_t xSpiMutex = NULL;

bool smsSentForBreach = false;
uint8_t currentPwmDuty = 0;
TickType_t xLastActivityTime = 0;
bool isLoRaReady = false;

char subscribers[MAX_SUBSCRIBERS][PHONE_LEN];
uint8_t subscriberCount = 0;

void loadSubscribers() {
  Serial.println(F("[NVS] Loading subscribers..."));
  try {
    if (!preferences.begin("gsm_subs", true)) {
      Serial.println(F("[NVS ERROR] Failed to open Preferences namespace. Using defaults."));
      strncpy(subscribers[0], TARGET_PHONE_NUM, PHONE_LEN - 1);
      subscribers[0][PHONE_LEN - 1] = '\0';
      subscriberCount = 1;
      return;
    }
    
    subscriberCount = preferences.getUChar("count", 0);
    if (subscriberCount == 0) {
      strncpy(subscribers[0], TARGET_PHONE_NUM, PHONE_LEN - 1);
      subscribers[0][PHONE_LEN - 1] = '\0';
      subscriberCount = 1;
      Serial.printf("[NVS] No saved numbers. Default added: %s\n", subscribers[0]);
    } else {
      for (uint8_t i = 0; i < subscriberCount; i++) {
        String key = "sub" + String(i);
        String num = preferences.getString(key.c_str(), "");
        strncpy(subscribers[i], num.c_str(), PHONE_LEN - 1);
        subscribers[i][PHONE_LEN - 1] = '\0';
        Serial.printf("[NVS] Sub [%d]: %s\n", i, subscribers[i]);
      }
    }
    preferences.end();
  } catch (...) {
    Serial.println(F("[NVS CRITICAL] Exception thrown during subscriber read! Using fallback."));
    strncpy(subscribers[0], TARGET_PHONE_NUM, PHONE_LEN - 1);
    subscribers[0][PHONE_LEN - 1] = '\0';
    subscriberCount = 1;
  }
}

bool addSubscriberNVS(const char* newPhone) {
  if (!newPhone || strlen(newPhone) == 0) return false;
  Serial.printf("[NVS] Attempting to add subscriber: %s\n", newPhone);
  if (subscriberCount >= MAX_SUBSCRIBERS) {
    Serial.println(F("[NVS ERROR] Maximum subscriber count reached!"));
    return false;
  }
  for (uint8_t i = 0; i < subscriberCount; i++) {
    if (strcmp(subscribers[i], newPhone) == 0) {
      Serial.println(F("[NVS] Phone number already exists."));
      return true;
    }
  }
  try {
    if (preferences.begin("gsm_subs", false)) {
      String key = "sub" + String(subscriberCount);
      preferences.putString(key.c_str(), newPhone);
      strncpy(subscribers[subscriberCount], newPhone, PHONE_LEN - 1);
      subscribers[subscriberCount][PHONE_LEN - 1] = '\0';
      subscriberCount++;
      preferences.putUChar("count", subscriberCount);
      preferences.end();
      Serial.printf("[NVS SUCCESS] Added sub #%d: %s\n", subscriberCount, newPhone);
      return true;
    }
  } catch (...) {
    Serial.println(F("[NVS ERROR] Exception during subscriber write operation."));
  }
  return false;
}

void clearSubscribersNVS() {
  Serial.println(F("[NVS] Clearing all saved subscribers..."));
  try {
    if (preferences.begin("gsm_subs", false)) {
      preferences.clear();
      preferences.putUChar("count", 0);
      preferences.end();
      subscriberCount = 0;
      Serial.println(F("[NVS SUCCESS] Subscribers cleared."));
    }
  } catch (...) {
    Serial.println(F("[NVS ERROR] Failed to clear subscribers table."));
  }
}

void resetInactivityTimer() {
  if (xActivityMutex != NULL && xSemaphoreTake(xActivityMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    xLastActivityTime = xTaskGetTickCount();
    xSemaphoreGive(xActivityMutex);
  }
}

TickType_t getLastActivityTime() {
  TickType_t temp = 0;
  if (xActivityMutex != NULL && xSemaphoreTake(xActivityMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    temp = xLastActivityTime;
    xSemaphoreGive(xActivityMutex);
  }
  return temp;
}

void setSystemModeAtomic(EventBits_t targetModeBit) {
  if (xSystemEvents == NULL) return;
  xEventGroupClearBits(xSystemEvents, ALL_MODES_MASK);
  xEventGroupSetBits(xSystemEvents, targetModeBit);
  resetInactivityTimer();
  Serial.printf("[SYSTEM] Mode changed to: %s\n", (targetModeBit & MODE_OVERRIDE_BIT) ? "OVERRIDE" : "INSPECT/MONITOR");
}

void initTinyML() {
  Serial.println(F("[TinyML] Initializing TensorFlow Lite Micro..."));
  isTinyMlReady = false;
  try {
    static tflite::MicroErrorReporter microErrorReporter;
    errorReporter = &microErrorReporter;
  #ifdef g_model_data
    model = tflite::GetModel(g_model_data);
  #else
    model = tflite::GetModel(g_model);
  #endif
    if (model->version() != TFLITE_SCHEMA_VERSION) {
      Serial.printf("[TinyML ERROR] Schema version mismatch! Expected %d, got %ld\n", TFLITE_SCHEMA_VERSION, model->version());
      return;
    }
    static tflite::AllOpsResolver resolver;
    static tflite::MicroInterpreter static_interpreter(model, resolver, tensorArena, kTensorArenaSize, errorReporter);
    interpreter = &static_interpreter;
    if (interpreter->AllocateTensors() != kTfLiteOk) {
      Serial.println(F("[TinyML ERROR] Tensor allocation failed!"));
      return;
    }
    input = interpreter->input(0);
    output = interpreter->output(0);
    isTinyMlReady = true;
    Serial.println(F("[TinyML SUCCESS] Model loaded & Tensors allocated successfully."));
  } catch (...) {
    Serial.println(F("[TinyML CRITICAL] Internal exception during execution setup!"));
    isTinyMlReady = false;
  }
}

void stopActuators() {
  if (currentPwmDuty > 0) {
    Serial.println(F("[ACTUATOR] Stopping Pump & Fan..."));
  }
  currentPwmDuty = 0;
  ledcWrite(PUMP_PWM_CHANNEL, 0);
  if (FAN_PIN != -1) ledcWrite(FAN_PWM_CHANNEL, 0);
}

void triggerDeepSleep() {
  Serial.println(F("[POWER] Shutdown initiated..."));
  Serial.flush();
  sysPowerState = false;

  vTaskDelete(xTaskGetHandle("LoRaTask"));
  vTaskDelete(xTaskGetHandle("SystemEngine"));
  vTaskDelete(xTaskGetHandle("GSM_Task"));
  vTaskDelete(xTaskGetHandle("ButtonTask"));

  if (xSpiMutex != NULL && xSemaphoreTake(xSpiMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
    if (isLoRaReady) LoRa.receive();
    xSemaphoreGive(xSpiMutex);
  }

  digitalWrite(GREEN_LED, LOW);
  digitalWrite(YELLOW_LED, LOW);
  digitalWrite(RED_LED, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  stopActuators();

  pinMode(POWER_BUTTON, INPUT_PULLUP);
  while (digitalRead(POWER_BUTTON) == LOW) { vTaskDelay(pdMS_TO_TICKS(10)); }
  vTaskDelay(pdMS_TO_TICKS(150));

  #if defined(LORA_DIO0_PIN) && (LORA_DIO0_PIN >= 0)
  rtc_gpio_init((gpio_num_t)LORA_DIO0_PIN);
  rtc_gpio_set_direction((gpio_num_t)LORA_DIO0_PIN, RTC_GPIO_MODE_INPUT_ONLY);
  gpio_pullup_dis((gpio_num_t)LORA_DIO0_PIN);
  gpio_pulldown_en((gpio_num_t)LORA_DIO0_PIN);
  esp_sleep_enable_ext0_wakeup((gpio_num_t)LORA_DIO0_PIN, 1);
  #endif

  rtc_gpio_init((gpio_num_t)POWER_BUTTON);
  rtc_gpio_set_direction((gpio_num_t)POWER_BUTTON, RTC_GPIO_MODE_INPUT_ONLY);
  rtc_gpio_pullup_en((gpio_num_t)POWER_BUTTON);
  rtc_gpio_pulldown_dis((gpio_num_t)POWER_BUTTON);
  esp_sleep_enable_ext1_wakeup(BUTTON_WAKEUP_MASK, ESP_EXT1_WAKEUP_ALL_LOW);

  Serial.println(F("[POWER] Entering Deep Sleep NOW."));
  Serial.flush();
  esp_deep_sleep_start();
}

void vTaskButtonHandler(void *pvParameters) {
  Serial.println(F("[TASK] Button Task Started."));
  pinMode(MODE_BUTTON,  INPUT_PULLUP);
  pinMode(POWER_BUTTON, INPUT_PULLUP);

  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT1) {
    Serial.println(F("[WAKEUP] Caused by POWER button press (EXT1)."));
    while (digitalRead(POWER_BUTTON) == LOW) { vTaskDelay(pdMS_TO_TICKS(20)); }
    vTaskDelay(pdMS_TO_TICKS(200));
  } else if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0) {
    Serial.println(F("[WAKEUP] Caused by LoRa DIO0 IRQ (EXT0)."));
  }

  TickType_t lastDebounceTime = xTaskGetTickCount();
  const TickType_t debounceDelay = pdMS_TO_TICKS(400);

  for (;;) {
    TickType_t now = xTaskGetTickCount();
    if ((now - lastDebounceTime) > debounceDelay) {
      if (digitalRead(MODE_BUTTON) == LOW) {
        Serial.println(F("[BUTTON] Mode Button Pressed!"));
        SystemCommandType sysCmd = CMD_TOGGLE_MODE;
        xQueueSend(systemCmdQueue, &sysCmd, 0);
        lastDebounceTime = now;
      } else if (digitalRead(POWER_BUTTON) == LOW) {
        Serial.println(F("[BUTTON] Power Button Pressed!"));
        SystemCommandType sysCmd = CMD_TRIGGER_SLEEP;
        xQueueSend(systemCmdQueue, &sysCmd, 0);
        lastDebounceTime = now;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void vTaskPowerManagement(void *pvParameters) {
  Serial.println(F("[TASK] Power Management Task Started."));
  resetInactivityTimer();
  const TickType_t xAwakeTimeoutTicks = pdMS_TO_TICKS(AWAKE_INTERVAL_SEC * 1000);

  for (;;) {
    if (xSemaphoreTake(xPowerSleepSemaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
      Serial.println(F("[POWER] Sleep semaphore triggered. Shutting down..."));
      triggerDeepSleep();
    }
    if ((xTaskGetTickCount() - getLastActivityTime()) >= xAwakeTimeoutTicks) {
      EventBits_t bits = xEventGroupGetBits(xSystemEvents);
      if (!(bits & THRESHOLD_BREACH_BIT) && !(bits & MODE_OVERRIDE_BIT)) {
        Serial.println(F("[POWER] Inactivity timeout reached. Entering deep sleep..."));
        triggerDeepSleep();
      } else {
        resetInactivityTimer();
      }
    }
  }
}

bool initLoRaRadio() {
  if (xSpiMutex != NULL && xSemaphoreTake(xSpiMutex, portMAX_DELAY) == pdTRUE) {
    SPI.begin(LORA_SCK_PIN, LORA_MISO_PIN, LORA_MOSI_PIN, LORA_CS_PIN);
    LoRa.setPins(LORA_CS_PIN, LORA_RST_PIN, LORA_DIO0_PIN);
    if (!LoRa.begin(LORA_FREQ)) {
      Serial.println(F("[LoRa ERROR] Radio Hardware Init Failed!"));
      isLoRaReady = false;
      xSemaphoreGive(xSpiMutex);
      return false;
    }
    LoRa.setSpreadingFactor(7);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setCodingRate4(5);
    LoRa.setSyncWord(0x12);
    LoRa.enableCrc();
    LoRa.receive();
    isLoRaReady = true;
    xSemaphoreGive(xSpiMutex);
    Serial.println(F("[LoRa SUCCESS] Radio Initialized (868MHz, SyncWord: 0x12). Listening..."));
    return true;
  }
  return false;
}

void vTaskLoRaEngine(void *pvParameters) {
  Serial.println(F("[TASK] LoRa Engine Task Started."));
  if (!initLoRaRadio()) {
    Serial.println(F("[LoRa CRITICAL] Radio unavailable at launch. Polling for recovery..."));
  }

  char outboundPayload[160];

  for (;;) {
    if (!isLoRaReady) {
      vTaskDelay(pdMS_TO_TICKS(5000));
      initLoRaRadio();
      continue;
    }

    // Outbound Transmit
    if (xQueueReceive(loraTxQueue, outboundPayload, 0) == pdTRUE) {
      if (xSpiMutex != NULL && xSemaphoreTake(xSpiMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        try {
          Serial.printf("[LoRa TX START] Transmitting: %s\n", outboundPayload);
          LoRa.idle();
          LoRa.beginPacket();
          LoRa.print(outboundPayload);
          if (LoRa.endPacket()) {
            Serial.println(F("[LoRa TX COMPLETE] Packet dispatched successfully."));
          } else {
            Serial.println(F("[LoRa TX ERROR] Transmission write failed!"));
          }
          LoRa.receive();
        } catch (...) {
          Serial.println(F("[LoRa TX EXCEPTION] Transmit operation failed safely."));
        }
        xSemaphoreGive(xSpiMutex);
      }
    }

    // Inbound Receive
    if (xSpiMutex != NULL && xSemaphoreTake(xSpiMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      try {
        int packetSize = LoRa.parsePacket();
        if (packetSize > 0) {
          resetInactivityTimer();
          String payload = "";
          while (LoRa.available()) payload += (char)LoRa.read();

          Serial.printf("[LoRa RX] Received %d bytes (RSSI: %d, SNR: %.1f): %s\n", 
                        packetSize, LoRa.packetRssi(), LoRa.packetSnr(), payload.c_str());

          JsonDocument doc;
          DeserializationError err = deserializeJson(doc, payload);

          if (!err && doc["cmd"].is<const char*>()) {
            const char* cmd = doc["cmd"];
            const char* val = doc["val"] | "";
            Serial.printf("[LoRa CMD] Parsed Command -> cmd: '%s', val: '%s'\n", cmd, val);

            if (strcmp(cmd, "SET_MODE") == 0) {
              SystemCommandType sysCmd = (strcmp(val, "GUARDIAN") == 0 || strcmp(val, "OVERRIDE") == 0) ? CMD_SET_MODE_OVERRIDE : CMD_SET_MODE_INSPECT;
              xQueueSend(systemCmdQueue, &sysCmd, 0);
            } else if (strcmp(cmd, "POWER") == 0 && (strcmp(val, "OFF") == 0 || strcmp(val, "TOGGLE") == 0)) {
              SystemCommandType sysCmd = CMD_TRIGGER_SLEEP;
              xQueueSend(systemCmdQueue, &sysCmd, 0);
            } else if (strcmp(cmd, "GSM_SEND_SMS") == 0 && strlen(val) > 0) {
              char smsBuf[160];
              snprintf(smsBuf, sizeof(smsBuf), "[REMOTE] %s", val);
              xQueueSend(gsmQueue, smsBuf, 0);
            } else if (strcmp(cmd, "GSM_ADD_SUB") == 0 && strlen(val) > 0) {
              addSubscriberNVS(val);
            } else if (strcmp(cmd, "GSM_CLEAR_SUBS") == 0) {
              clearSubscribersNVS();
            }
          } else if (err) {
            Serial.printf("[LoRa RX ERROR] Deserialization failed: %s\n", err.c_str());
          }
        }
      } catch (...) {
        Serial.println(F("[LoRa RX EXCEPTION] Packet read failure ignored."));
      }
      xSemaphoreGive(xSpiMutex);
    }
    vTaskDelay(pdMS_TO_TICKS(40));
  }
}

void vTaskGSM(void *pvParameters) {
  Serial.println(F("[TASK] GSM Task Started."));
  resetInactivityTimer();
  
  gsmSerial.begin(9600, SERIAL_8N1, GSM_RX_PIN, GSM_TX_PIN);
  vTaskDelay(pdMS_TO_TICKS(1000));
  
  gsmSerial.println("AT");
  vTaskDelay(pdMS_TO_TICKS(300));
  gsmSerial.println("AT+CMGF=1");
  vTaskDelay(pdMS_TO_TICKS(300));
  Serial.println(F("[GSM] Serial Interface Ready (9600 Baud, Text Mode)."));

  char alertMessage[160];
  for (;;) {
    if (xQueueReceive(gsmQueue, alertMessage, pdMS_TO_TICKS(100)) == pdTRUE) {
      Serial.printf("[GSM] SMS Dispatch requested: \"%s\"\n", alertMessage);
      
      for (uint8_t i = 0; i < subscriberCount; i++) {
        Serial.printf("[GSM TX] Sending to sub [%d/%d]: %s\n", i + 1, subscriberCount, subscribers[i]);
        
        try {
          gsmSerial.println("AT+CMGF=1");
          vTaskDelay(pdMS_TO_TICKS(200));
          gsmSerial.printf("AT+CMGS=\"%s\"\r\n", subscribers[i]);
          vTaskDelay(pdMS_TO_TICKS(300));
          gsmSerial.print(alertMessage);
          gsmSerial.write(26); // CTRL+Z
          
          TickType_t startWait = xTaskGetTickCount();
          bool sendOk = false;
          while ((xTaskGetTickCount() - startWait) < pdMS_TO_TICKS(5000)) {
            if (gsmSerial.available()) {
              String resp = gsmSerial.readString();
              if (resp.indexOf("OK") != -1 || resp.indexOf("+CMGS:") != -1) {
                sendOk = true;
                break;
              }
            }
            vTaskDelay(pdMS_TO_TICKS(100));
          }

          if (sendOk) {
            Serial.println(F("[GSM TX SUCCESS] Message confirmed by network."));
          } else {
            Serial.println(F("[GSM TX TIMEOUT] No confirmation received within interval."));
          }
        } catch (...) {
          Serial.println(F("[GSM ERROR] Execution failed during message transfer."));
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void vTaskSystemEngine(void *pvParameters) {
  Serial.println(F("[TASK] System Engine Task Started."));
  TickType_t xLastDhtReadTime = xTaskGetTickCount();
  TickType_t xLastTxWindow    = xTaskGetTickCount();
  const TickType_t xDhtInterval = pdMS_TO_TICKS(2000);

  SystemCommandType inboundCmd;

  static float filteredTemp = -1.0f;
  static float filteredHum  = -1.0f;
  static float filteredVoc  = -1.0f;

  static double tempSum = 0, humSum = 0, vocSum = 0;
  static uint16_t samplesInWindow = 0;

  for (;;) {
    if (xQueueReceive(systemCmdQueue, &inboundCmd, 0) == pdTRUE) {
      resetInactivityTimer();
      Serial.printf("[ENGINE] Command Received: %d\n", (int)inboundCmd);
      switch (inboundCmd) {
        case CMD_SET_MODE_INSPECT:  setSystemModeAtomic(MODE_INSPECT_BIT); break;
        case CMD_SET_MODE_OVERRIDE: setSystemModeAtomic(MODE_OVERRIDE_BIT); break;
        case CMD_TOGGLE_MODE: {
          EventBits_t currentBits = xEventGroupGetBits(xSystemEvents);
          setSystemModeAtomic((currentBits & MODE_OVERRIDE_BIT) ? MODE_INSPECT_BIT : MODE_OVERRIDE_BIT);
          break;
        }
        case CMD_TRIGGER_SLEEP:
          if (xPowerSleepSemaphore != NULL) xSemaphoreGive(xPowerSleepSemaphore);
          break;
      }
    }

    TickType_t xNow = xTaskGetTickCount();
    if ((xNow - xLastDhtReadTime) >= xDhtInterval) {
      xLastDhtReadTime = xNow;
      
      float rawTemp = NAN;
      float rawHum  = NAN;
      
      try {
        rawTemp = dht.readTemperature();
        rawHum  = dht.readHumidity();
      } catch (...) {
        Serial.println(F("[DHT EXCEPTION] Bus access failure during sensor conversion."));
      }
      
      if (!isnan(rawTemp) && rawTemp >= -40.0f && rawTemp <= 80.0f) {
        filteredTemp = (filteredTemp < 0.0f) ? rawTemp : (EMA_ALPHA * rawTemp + (1.0f - EMA_ALPHA) * filteredTemp);
      } else {
        Serial.println(F("[SENSOR WARNING] DHT Temperature read invalid!"));
      }

      if (!isnan(rawHum) && rawHum >= 0.0f && rawHum <= 100.0f) {
        filteredHum = (filteredHum < 0.0f) ? rawHum : (EMA_ALPHA * rawHum + (1.0f - EMA_ALPHA) * filteredHum);
      } else {
        Serial.println(F("[SENSOR WARNING] DHT Humidity read invalid!"));
      }
    }

    int rawVoc = -1;
    try {
      rawVoc = analogRead(MQ2_PIN);
    } catch (...) {
      Serial.println(F("[MQ2 EXCEPTION] Analog read failure!"));
    }

    if (rawVoc >= 0 && rawVoc <= 4095) {
      filteredVoc = (filteredVoc < 0.0f) ? (float)rawVoc : (EMA_ALPHA * (float)rawVoc + (1.0f - EMA_ALPHA) * filteredVoc);
    }

    // Accumulate metrics if gas sensor OR DHT sensors return valid data
    if (filteredVoc >= 0.0f || (filteredTemp >= 0.0f && filteredHum >= 0.0f)) {
      tempSum += (filteredTemp >= 0.0f) ? filteredTemp : 0.0f;
      humSum  += (filteredHum >= 0.0f)  ? filteredHum  : 0.0f;
      vocSum  += (filteredVoc >= 0.0f)  ? filteredVoc  : 0.0f;
      samplesInWindow++;
    }

    // Outbound LoRa Transmission Dispatch
    if ((xNow - xLastTxWindow) >= pdMS_TO_TICKS(TX_FAST_INTERVAL_MS)) {
      xLastTxWindow = xNow;

      if (samplesInWindow > 0) {
        EventBits_t currentBits = xEventGroupGetBits(xSystemEvents);
        bool isOverride = (currentBits & MODE_OVERRIDE_BIT) != 0;
        bool isBreached = (currentBits & THRESHOLD_BREACH_BIT) != 0;

        JsonDocument doc;
        doc["temp"]     = (filteredTemp >= 0.0f) ? (float)(tempSum / samplesInWindow) : -999.0f;
        doc["hum"]      = (filteredHum  >= 0.0f) ? (float)(humSum / samplesInWindow)  : -999.0f;
        doc["voc"]      = (filteredVoc  >= 0.0f) ? (int)(vocSum / samplesInWindow)    : -1;
        doc["samples"]  = samplesInWindow;
        doc["mode"]     = isOverride ? "OVERRIDE" : (isBreached ? "GUARDIAN" : "MONITOR");
        doc["breach"]   = isBreached;
        doc["dht_err"]  = (filteredTemp < 0.0f || filteredHum < 0.0f);
        doc["gsm_subs"] = subscriberCount;

        char jsonBuffer[160];
        serializeJson(doc, jsonBuffer, sizeof(jsonBuffer));

        Serial.printf("[TELEMETRY] Queuing LoRa TX (%d samples): %s\n", samplesInWindow, jsonBuffer);

        if (xQueueSend(loraTxQueue, jsonBuffer, pdMS_TO_TICKS(50)) != pdTRUE) {
          Serial.println(F("[SystemEngine ERROR] loraTxQueue FULL! Dropping message."));
        }

        tempSum = 0; humSum = 0; vocSum = 0; samplesInWindow = 0;
      } else {
        Serial.println(F("[TELEMETRY WARNING] Zero valid samples accumulated. Skipping TX packet."));
      }
    }

    bool breachDetected = false;
    float modelScore = 0.0f;

    // Run inference only if TinyML engine is fully functional AND DHT measurements exist
    if (isTinyMlReady && filteredTemp >= 0.0f && filteredHum >= 0.0f && filteredVoc >= 0.0f) {
      try {
        input->data.f[0] = filteredTemp / 100.0f;
        input->data.f[1] = filteredHum / 100.0f;
        input->data.f[2] = filteredVoc / 4095.0f;

        if (interpreter->Invoke() == kTfLiteOk) {
          modelScore = output->data.f[1];
          if (modelScore > 0.75f) breachDetected = true;
        } else {
          Serial.println(F("[TinyML ERROR] Model invocation failed during execution!"));
        }
      } catch (...) {
        Serial.println(F("[TinyML CRITICAL] Standard exception encountered during inference!"));
      }
    } else {
      // Fallback thresholding algorithm if DHT22 or TinyML fails
      if ((filteredVoc > VOC_THRESHOLD && filteredVoc >= 0.0f) || 
          (filteredTemp > TEMP_THRESHOLD && filteredTemp >= 0.0f) || 
          (filteredHum > HUM_THRESHOLD && filteredHum >= 0.0f)) {
        breachDetected = true;
      }
    }

    if (breachDetected) xEventGroupSetBits(xSystemEvents, THRESHOLD_BREACH_BIT);
    else xEventGroupClearBits(xSystemEvents, THRESHOLD_BREACH_BIT);

    EventBits_t currentBits = xEventGroupGetBits(xSystemEvents);
    bool isOverride = (currentBits & MODE_OVERRIDE_BIT) != 0;
    bool isBreached = (currentBits & THRESHOLD_BREACH_BIT) != 0;

    if (isOverride) {
      digitalWrite(GREEN_LED, LOW); digitalWrite(YELLOW_LED, HIGH); digitalWrite(RED_LED, LOW); digitalWrite(BUZZER_PIN, HIGH);
      if (currentPwmDuty < 255) {
        currentPwmDuty = (currentPwmDuty + 25 > 255) ? 255 : currentPwmDuty + 25;
        ledcWrite(PUMP_PWM_CHANNEL, currentPwmDuty);
        if (FAN_PIN != -1) ledcWrite(FAN_PWM_CHANNEL, currentPwmDuty);
      }
      smsSentForBreach = false;
    } else if (isBreached) {
      digitalWrite(GREEN_LED, LOW); digitalWrite(YELLOW_LED, LOW); digitalWrite(RED_LED, HIGH); digitalWrite(BUZZER_PIN, HIGH);
      if (currentPwmDuty < 255) {
        currentPwmDuty = (currentPwmDuty + 25 > 255) ? 255 : currentPwmDuty + 25;
        ledcWrite(PUMP_PWM_CHANNEL, currentPwmDuty);
        if (FAN_PIN != -1) ledcWrite(FAN_PWM_CHANNEL, currentPwmDuty);
      }
      if (!smsSentForBreach) {
        char msg[160];
        snprintf(msg, sizeof(msg), "ALARM BREACH!\r\nT:%.1fC H:%.1f%% VOC:%d", 
                 (filteredTemp >= 0.0f ? filteredTemp : -1.0f), 
                 (filteredHum >= 0.0f ? filteredHum : -1.0f), 
                 (int)filteredVoc);
        Serial.printf("[ALARM] Threshold breach detected! Score: %.2f. Queuing SMS Alert...\n", modelScore);
        if (xQueueSend(gsmQueue, msg, 0) == pdTRUE) smsSentForBreach = true;
      }
    } else {
      digitalWrite(GREEN_LED, HIGH); digitalWrite(YELLOW_LED, LOW); digitalWrite(RED_LED, LOW); digitalWrite(BUZZER_PIN, LOW);
      stopActuators();
      smsSentForBreach = false;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println(F("\n=========================================="));
  Serial.println(F("     ESP32 GRAIN MONITOR NODE STARTING    "));
  Serial.println(F("=========================================="));

  rtc_gpio_deinit((gpio_num_t)POWER_BUTTON);
  #if defined(LORA_DIO0_PIN) && (LORA_DIO0_PIN >= 0)
  rtc_gpio_deinit((gpio_num_t)LORA_DIO0_PIN);
  #endif

  Serial.println(F("[SETUP] Initializing NVS Flash..."));
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);

  loadSubscribers();

  Serial.println(F("[SETUP] Configuring GPIOs & PWM Channels..."));
  pinMode(GREEN_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcAttachChannel(PUMP_PIN, PWM_FREQ, PWM_RES, PUMP_PWM_CHANNEL);
  if (FAN_PIN != -1) ledcAttachChannel(FAN_PIN, PWM_FREQ, PWM_RES, FAN_PWM_CHANNEL);
#else
  ledcSetup(PUMP_PWM_CHANNEL, PWM_FREQ, PWM_RES);
  ledcAttachPin(PUMP_PIN, PUMP_PWM_CHANNEL);
  if (FAN_PIN != -1) {
    ledcSetup(FAN_PWM_CHANNEL, PWM_FREQ, PWM_RES);
    ledcAttachPin(FAN_PIN, FAN_PWM_CHANNEL);
  }
#endif

  Serial.println(F("[SETUP] Creating FreeRTOS Synchronization Objects..."));
  xSystemEvents = xEventGroupCreate();
  xPowerSleepSemaphore = xSemaphoreCreateBinary();
  xActivityMutex = xSemaphoreCreateMutex();
  xSpiMutex = xSemaphoreCreateMutex();   

  gsmQueue       = xQueueCreate(3, sizeof(char[160]));
  systemCmdQueue = xQueueCreate(10, sizeof(SystemCommandType));
  loraTxQueue    = xQueueCreate(5, sizeof(char[160]));

  setSystemModeAtomic(MODE_INSPECT_BIT);
  dht.begin();
  initTinyML();

  Serial.println(F("[SETUP] Launching FreeRTOS Tasks..."));
  xTaskCreatePinnedToCore(vTaskPowerManagement, "PowerTask",    3072,  NULL, 3, NULL, 0);
  xTaskCreatePinnedToCore(vTaskButtonHandler,    "ButtonTask",   2048,  NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(vTaskSystemEngine,     "SystemEngine", 10240, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(vTaskLoRaEngine,       "LoRaTask",     8192,  NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(vTaskGSM,              "GSM_Task",      4096,  NULL, 1, NULL, 1);

  Serial.println(F("[SETUP COMPLETE] System initialization finished successfully."));
}

void loop() {
  vTaskDelete(NULL);
}