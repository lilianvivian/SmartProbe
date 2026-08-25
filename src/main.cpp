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

// --- TinyML TensorFlow Lite Headers ---
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

// --- Configuration Constants ---
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
#define SLEEP_DURATION_SEC   (3 * 60)
#define SLEEP_TIMER_US       (SLEEP_DURATION_SEC * 1000000ULL)

#define TX_INTERVAL_MS       (30 * 1000) // Transmit batch metrics every 30 seconds
#define EMA_ALPHA            0.2f

#define MAX_SUBSCRIBERS      5
#define PHONE_LEN            16


// Deep-sleep wakeup mask ONLY for POWER_BUTTON (GPIO 15)
#define BUTTON_WAKEUP_MASK   (1ULL << POWER_BUTTON)

// --- TinyML Arena ---
namespace {
  tflite::ErrorReporter* errorReporter = nullptr;
  const tflite::Model* model = nullptr;
  tflite::MicroInterpreter* interpreter = nullptr;
  TfLiteTensor* input = nullptr;
  TfLiteTensor* output = nullptr;
  constexpr int kTensorArenaSize = 8 * 1024;
  uint8_t tensorArena[kTensorArenaSize];
}

enum SystemCommandType {
  CMD_SET_MODE_INSPECT,
  CMD_SET_MODE_OVERRIDE,
  CMD_TOGGLE_MODE,
  CMD_TRIGGER_SLEEP
};

struct SensorData {
  float temp;
  float humidity;
  int voc;
  uint16_t sampleCount;
  bool readyToTx;
};

// --- Global Objects ---
DHT dht(DHT_PIN, DHT_TYPE);
HardwareSerial gsmSerial(2);
Preferences preferences;

EventGroupHandle_t xSystemEvents = NULL;
QueueHandle_t gsmQueue = NULL;
QueueHandle_t systemCmdQueue = NULL;
SemaphoreHandle_t xPowerSleepSemaphore = NULL;
SemaphoreHandle_t xActivityMutex = NULL;
SemaphoreHandle_t xSpiMutex = NULL;

SensorData latestMetrics = {-1.0f, -1.0f, 0, 0, false};
bool smsSentForBreach = false;
uint8_t currentPwmDuty = 0;
TickType_t xLastActivityTime = 0;

char subscribers[MAX_SUBSCRIBERS][PHONE_LEN];
uint8_t subscriberCount = 0;

// --- Helper Functions for GSM NVS Management ---
void loadSubscribers() {
  preferences.begin("gsm_subs", true);
  subscriberCount = preferences.getUChar("count", 0);
  
  if (subscriberCount == 0) {
    strncpy(subscribers[0], TARGET_PHONE_NUM, PHONE_LEN - 1);
    subscribers[0][PHONE_LEN - 1] = '\0';
    subscriberCount = 1;
  } else {
    for (uint8_t i = 0; i < subscriberCount; i++) {
      String key = "sub" + String(i);
      String num = preferences.getString(key.c_str(), "");
      strncpy(subscribers[i], num.c_str(), PHONE_LEN - 1);
      subscribers[i][PHONE_LEN - 1] = '\0';
    }
  }
  preferences.end();
}

bool addSubscriberNVS(const char* newPhone) {
  if (subscriberCount >= MAX_SUBSCRIBERS) return false;

  for (uint8_t i = 0; i < subscriberCount; i++) {
    if (strcmp(subscribers[i], newPhone) == 0) return true;
  }

  preferences.begin("gsm_subs", false);
  String key = "sub" + String(subscriberCount);
  preferences.putString(key.c_str(), newPhone);
  strncpy(subscribers[subscriberCount], newPhone, PHONE_LEN - 1);
  subscribers[subscriberCount][PHONE_LEN - 1] = '\0';
  
  subscriberCount++;
  preferences.putUChar("count", subscriberCount);
  preferences.end();
  
  Serial.printf("[GSM NVS] Added subscriber #%d: %s\n", subscriberCount, newPhone);
  return true;
}

void clearSubscribersNVS() {
  preferences.begin("gsm_subs", false);
  preferences.clear();
  preferences.putUChar("count", 0);
  preferences.end();
  
  subscriberCount = 0;
  Serial.println(F("[GSM NVS] Cleared all subscriber entries."));
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
}

void initTinyML() {
  static tflite::MicroErrorReporter microErrorReporter;
  errorReporter = &microErrorReporter;
  
#ifdef g_model_data
  model = tflite::GetModel(g_model_data);
#else
  model = tflite::GetModel(g_model);
#endif

  if (model->version() != TFLITE_SCHEMA_VERSION) return;

  static tflite::AllOpsResolver resolver;
  static tflite::MicroInterpreter static_interpreter(
      model, resolver, tensorArena, kTensorArenaSize, errorReporter);
  interpreter = &static_interpreter;

  if (interpreter->AllocateTensors() != kTfLiteOk) return;
  input = interpreter->input(0);
  output = interpreter->output(0);
}

void stopActuators() {
  currentPwmDuty = 0;
  ledcWrite(PUMP_PWM_CHANNEL, 0);
  if (FAN_PIN != -1) ledcWrite(FAN_PWM_CHANNEL, 0);
}

void triggerDeepSleep() {
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(YELLOW_LED, LOW);
  digitalWrite(RED_LED, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  stopActuators();

  if (xSpiMutex != NULL && xSemaphoreTake(xSpiMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
    LoRa.sleep();
    xSemaphoreGive(xSpiMutex);
  }

  pinMode(POWER_BUTTON, INPUT_PULLUP);
  while (digitalRead(POWER_BUTTON) == LOW) {
    vTaskDelay(pdMS_TO_TICKS(10)); // Block sleep until finger is off the button
  }
  vTaskDelay(pdMS_TO_TICKS(100));
  
  // 1. Timer Wakeup
  esp_sleep_enable_timer_wakeup(SLEEP_TIMER_US);
  
  // 2. Radio Signal Wakeup (DIO0)
  #if defined(LORA_DIO0_PIN) && (LORA_DIO0_PIN >= 0)
  gpio_pullup_dis((gpio_num_t)LORA_DIO0_PIN);
  gpio_pulldown_en((gpio_num_t)LORA_DIO0_PIN);
  esp_sleep_enable_ext0_wakeup((gpio_num_t)LORA_DIO0_PIN, 1);
  #endif

  // 3. Ext1 Single-Pin Wakeup ONLY for Power Button (Active LOW on GPIO 15)
  rtc_gpio_pullup_en((gpio_num_t)POWER_BUTTON);
  rtc_gpio_pulldown_dis((gpio_num_t)POWER_BUTTON);

  // Wake up strictly when POWER_BUTTON (GPIO 15) pulls to GND
  esp_sleep_enable_ext1_wakeup(BUTTON_WAKEUP_MASK, ESP_EXT1_WAKEUP_ALL_LOW);
  
  Serial.println(F("[POWER] Entering Deep Sleep (Waiting ONLY for Power Button on GPIO 15 -> GND)..."));
  Serial.flush();
  esp_deep_sleep_start();
}

void checkWakeupReason() {
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0: 
      Serial.println(F("[POWER] Wakeup Event: LoRa Radio Signal")); 
      break;
    case ESP_SLEEP_WAKEUP_EXT1: 
      Serial.println(F("[POWER] Wakeup Event: Power Button Press (GPIO 15)")); 
      break;
    case ESP_SLEEP_WAKEUP_TIMER: 
      Serial.println(F("[POWER] Wakeup Event: Scheduled Timer Cycle")); 
      break;
    default: 
      Serial.println(F("[POWER] Cold Boot / Hardware Reset")); 
      break;
  }
}

// --- Task: 2-Button Polling Task (Internal Pull-Up Active-LOW) ---
void vTaskButtonHandler(void *pvParameters) {
  // Use INPUT_PULLUP since buttons connect directly to GND
  pinMode(MODE_BUTTON,  INPUT_PULLUP); // GPIO 47
  pinMode(POWER_BUTTON, INPUT_PULLUP); // GPIO 15

  TickType_t lastDebounceTime = 0;
  const TickType_t debounceDelay = pdMS_TO_TICKS(1500);

  for (;;) {
    TickType_t now = xTaskGetTickCount();

    if ((now - lastDebounceTime) > debounceDelay) {
      if (digitalRead(MODE_BUTTON) == LOW) {
        SystemCommandType sysCmd = CMD_TOGGLE_MODE;
        xQueueSend(systemCmdQueue, &sysCmd, 0);
        Serial.println(F("[BUTTON] MODE Toggle Button Pressed (GPIO 47 -> GND)"));
        lastDebounceTime = now;
      } 
      else if (digitalRead(POWER_BUTTON) == LOW) {
        SystemCommandType sysCmd = CMD_TRIGGER_SLEEP;
        xQueueSend(systemCmdQueue, &sysCmd, 0);
        Serial.println(F("[BUTTON] POWER Button Pressed (GPIO 15 -> GND)"));
        lastDebounceTime = now;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void vTaskPowerManagement(void *pvParameters) {
  resetInactivityTimer();
  const TickType_t xAwakeTimeoutTicks = pdMS_TO_TICKS(AWAKE_INTERVAL_SEC * 1000);

  for (;;) {
    if (xSemaphoreTake(xPowerSleepSemaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
      triggerDeepSleep();
    }

    TickType_t xCurrentTicks = xTaskGetTickCount();
    if ((xCurrentTicks - getLastActivityTime()) >= xAwakeTimeoutTicks) {
      EventBits_t currentBits = xEventGroupGetBits(xSystemEvents);
      bool isBreached = (currentBits & THRESHOLD_BREACH_BIT) != 0;
      bool isOverride = (currentBits & MODE_OVERRIDE_BIT) != 0;

      if (!isBreached && !isOverride) {
        triggerDeepSleep();
      } else {
        resetInactivityTimer();
      }
    }
  }
}

// --- Task: LoRa Communication Engine ---
void vTaskLoRaEngine(void *pvParameters) {
  Serial.println(F("[LoRa] Re-initializing SPI & LoRa Radio on boot..."));
  
  if (xSpiMutex != NULL && xSemaphoreTake(xSpiMutex, portMAX_DELAY) == pdTRUE) {
    SPI.begin(LORA_SCK_PIN, LORA_MISO_PIN, LORA_MOSI_PIN, LORA_CS_PIN);
    LoRa.setPins(LORA_CS_PIN, LORA_RST_PIN, LORA_DIO0_PIN);

    if (!LoRa.begin(LORA_FREQ)) {
      Serial.println(F("[LoRa ERROR] Transceiver Initialization Failed!"));
      xSemaphoreGive(xSpiMutex);
      vTaskDelete(NULL);
    }

    LoRa.setSpreadingFactor(7);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setCodingRate4(5);
    LoRa.enableCrc();
    LoRa.idle();
    xSemaphoreGive(xSpiMutex);
  }

  Serial.println(F("[LoRa SUCCESS] Transceiver Ready & Listening @ 868MHz"));

  for (;;) {
    if (xSpiMutex != NULL && xSemaphoreTake(xSpiMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      
      // 1. Process Downlink Packet Reception
      int packetSize = LoRa.parsePacket();
      if (packetSize > 0) {
        resetInactivityTimer();
        String payload = "";
        payload.reserve(packetSize);
        while (LoRa.available()) payload += (char)LoRa.read();

        JsonDocument doc;
        DeserializationError err = deserializeJson(doc, payload);

        if (!err && doc["cmd"].is<const char*>()) {
          const char* cmd = doc["cmd"];
          const char* val = doc["val"] | "";

          Serial.printf("[LoRa RX] Cmd: '%s' | Val: '%s'\n", cmd, val);

          if (strcmp(cmd, "SET_MODE") == 0) {
            SystemCommandType sysCmd;
            if (strcmp(val, "GUARDIAN") == 0 || strcmp(val, "OVERRIDE") == 0) {
              sysCmd = CMD_SET_MODE_OVERRIDE;
              xQueueSend(systemCmdQueue, &sysCmd, 0);
            } else if (strcmp(val, "MONITOR") == 0 || strcmp(val, "INSPECT") == 0) {
              sysCmd = CMD_SET_MODE_INSPECT;
              xQueueSend(systemCmdQueue, &sysCmd, 0);
            }
          } 
          else if (strcmp(cmd, "POWER") == 0 && (strcmp(val, "TOGGLE") == 0 || strcmp(val, "OFF") == 0)) {
            SystemCommandType sysCmd = CMD_TRIGGER_SLEEP;
            xQueueSend(systemCmdQueue, &sysCmd, 0);
          }
          else if (strcmp(cmd, "GSM_SEND_SMS") == 0 && strlen(val) > 0) {
            char smsBuf[160];
            snprintf(smsBuf, sizeof(smsBuf), "[REMOTE LORA] %s", val);
            xQueueSend(gsmQueue, &smsBuf, 0);
          }
          else if (strcmp(cmd, "GSM_ADD_SUB") == 0 && strlen(val) > 0) {
            addSubscriberNVS(val);
          }
          else if (strcmp(cmd, "GSM_CLEAR_SUBS") == 0) {
            clearSubscribersNVS();
          }
        }
      }

      // 2. Transmit Upstream Metrics
      if (latestMetrics.readyToTx) {
        EventBits_t currentBits = xEventGroupGetBits(xSystemEvents);
        bool isOverride = (currentBits & MODE_OVERRIDE_BIT) != 0;
        bool isBreached = (currentBits & THRESHOLD_BREACH_BIT) != 0;

        JsonDocument doc;
        doc["temp"]     = latestMetrics.temp;
        doc["hum"]      = latestMetrics.humidity;
        doc["voc"]      = latestMetrics.voc;
        doc["samples"]  = latestMetrics.sampleCount;
        doc["mode"]     = isOverride ? "OVERRIDE" : (isBreached ? "GUARDIAN" : "MONITOR");
        doc["breach"]   = isBreached;
        doc["gsm_subs"] = subscriberCount;

        char jsonBuffer[160];
        size_t bytesSerialized = serializeJson(doc, jsonBuffer, sizeof(jsonBuffer));

        LoRa.beginPacket();
        LoRa.write((const uint8_t*)jsonBuffer, bytesSerialized);
        bool ok = LoRa.endPacket();

        Serial.printf("[LoRa TX] Averaged Payload (%u B): %s | Status: %s\n", 
                      bytesSerialized, jsonBuffer, ok ? "SUCCESS" : "FAILED");
        
        latestMetrics.readyToTx = false;
      }

      xSemaphoreGive(xSpiMutex);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void vTaskGSM(void *pvParameters) {
  resetInactivityTimer();
  gsmSerial.begin(9600, SERIAL_8N1, GSM_RX_PIN, GSM_TX_PIN);
  vTaskDelay(pdMS_TO_TICKS(1000));

  gsmSerial.println("AT");
  vTaskDelay(pdMS_TO_TICKS(500));
  gsmSerial.println("AT+CMGF=1");
  vTaskDelay(pdMS_TO_TICKS(500));

  char alertMessage[160];

  for (;;) {
    if (xQueueReceive(gsmQueue, &alertMessage, pdMS_TO_TICKS(100)) == pdTRUE) {
      for (uint8_t i = 0; i < subscriberCount; i++) {
        gsmSerial.println("AT+CMGF=1");
        vTaskDelay(pdMS_TO_TICKS(200));
        gsmSerial.printf("AT+CMGS=\"%s\"\r\n", subscribers[i]);
        vTaskDelay(pdMS_TO_TICKS(200));
        gsmSerial.print(alertMessage);
        gsmSerial.write(26);
        vTaskDelay(pdMS_TO_TICKS(3000));
      }
    }
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void vTaskSystemEngine(void *pvParameters) {
  TickType_t xLastDhtReadTime = xTaskGetTickCount();
  TickType_t xLast30SecWindow  = xTaskGetTickCount();
  const TickType_t xDhtInterval = pdMS_TO_TICKS(2000);
  
  SystemCommandType inboundCmd;

  static float filteredTemp = -1.0f;
  static float filteredHum  = -1.0f;
  static float filteredVoc  = -1.0f;

  static double tempSum = 0;
  static double humSum  = 0;
  static double vocSum  = 0;
  static uint16_t samplesInWindow = 0;

  for (;;) {
    if (xQueueReceive(systemCmdQueue, &inboundCmd, 0) == pdTRUE) {
      resetInactivityTimer();
      switch (inboundCmd) {
        case CMD_SET_MODE_INSPECT:
          setSystemModeAtomic(MODE_INSPECT_BIT);
          break;

        case CMD_SET_MODE_OVERRIDE:
          setSystemModeAtomic(MODE_OVERRIDE_BIT);
          break;

        case CMD_TOGGLE_MODE: {
          EventBits_t currentBits = xEventGroupGetBits(xSystemEvents);
          if ((currentBits & MODE_OVERRIDE_BIT) != 0) {
            setSystemModeAtomic(MODE_INSPECT_BIT);
          } else {
            setSystemModeAtomic(MODE_OVERRIDE_BIT);
          }
          break;
        }

        case CMD_TRIGGER_SLEEP:
          if (xPowerSleepSemaphore != NULL) xSemaphoreGive(xPowerSleepSemaphore);
          break;
      }
    }

    // --- 1. Real-time Sampling & EMA Filtering ---
    TickType_t xNow = xTaskGetTickCount();
    if ((xNow - xLastDhtReadTime) >= xDhtInterval) {
      xLastDhtReadTime = xNow;
      float rawTemp = dht.readTemperature();
      float rawHum  = dht.readHumidity();

      if (!isnan(rawTemp)) {
        filteredTemp = (filteredTemp < 0.0f) ? rawTemp : (EMA_ALPHA * rawTemp + (1.0f - EMA_ALPHA) * filteredTemp);
      }
      if (!isnan(rawHum)) {
        filteredHum = (filteredHum < 0.0f) ? rawHum : (EMA_ALPHA * rawHum + (1.0f - EMA_ALPHA) * filteredHum);
      }
    }

    int rawVoc = analogRead(MQ2_PIN);
    if (rawVoc >= 0 && rawVoc <= 4095) {
      filteredVoc = (filteredVoc < 0.0f) ? (float)rawVoc : (EMA_ALPHA * (float)rawVoc + (1.0f - EMA_ALPHA) * filteredVoc);
    }

    if (filteredTemp >= 0.0f && filteredHum >= 0.0f && filteredVoc >= 0.0f) {
      tempSum += filteredTemp;
      humSum  += filteredHum;
      vocSum  += filteredVoc;
      samplesInWindow++;
    }

    // --- 2. 30-Second Averaging & Telemetry Dispatch ---
    if ((xNow - xLast30SecWindow) >= pdMS_TO_TICKS(TX_INTERVAL_MS)) {
      xLast30SecWindow = xNow;

      if (samplesInWindow > 0) {
        latestMetrics.temp        = (float)(tempSum / samplesInWindow);
        latestMetrics.humidity    = (float)(humSum / samplesInWindow);
        latestMetrics.voc         = (int)(vocSum / samplesInWindow);
        latestMetrics.sampleCount = samplesInWindow;
        latestMetrics.readyToTx   = true;

        tempSum = 0; humSum = 0; vocSum = 0;
        samplesInWindow = 0;
      }
    }

    // --- 3. TinyML Inference / Rule Evaluation ---
    bool breachDetected = false;
    bool sensorsValid   = (filteredTemp >= 0.0f) && (filteredHum >= 0.0f);

    if (sensorsValid && interpreter != nullptr) {
      input->data.f[0] = filteredTemp / 100.0f;
      input->data.f[1] = filteredHum / 100.0f;
      input->data.f[2] = filteredVoc / 4095.0f;

      if (interpreter->Invoke() == kTfLiteOk) {
        if (output->data.f[1] > 0.75f) breachDetected = true;
      }
    } else {
      if (filteredVoc > VOC_THRESHOLD || 
         (filteredTemp > TEMP_THRESHOLD && filteredTemp >= 0.0f) || 
         (filteredHum > HUM_THRESHOLD && filteredHum >= 0.0f)) {
        breachDetected = true;
      }
    }

    if (breachDetected) xEventGroupSetBits(xSystemEvents, THRESHOLD_BREACH_BIT);
    else xEventGroupClearBits(xSystemEvents, THRESHOLD_BREACH_BIT);

    // --- 4. Actuator Control Logic ---
    EventBits_t currentBits = xEventGroupGetBits(xSystemEvents);
    bool isOverride = (currentBits & MODE_OVERRIDE_BIT) != 0;
    bool isBreached = (currentBits & THRESHOLD_BREACH_BIT) != 0;

    if (isOverride) {
      digitalWrite(GREEN_LED, LOW);
      digitalWrite(YELLOW_LED, HIGH);
      digitalWrite(RED_LED, LOW);
      digitalWrite(BUZZER_PIN, HIGH);
      if (currentPwmDuty < 255) {
        currentPwmDuty = (currentPwmDuty + 25 > 255) ? 255 : currentPwmDuty + 25;
        ledcWrite(PUMP_PWM_CHANNEL, currentPwmDuty);
        if (FAN_PIN != -1) ledcWrite(FAN_PWM_CHANNEL, currentPwmDuty);
      }
      smsSentForBreach = false;

    } else if (isBreached) {
      digitalWrite(GREEN_LED, LOW);
      digitalWrite(YELLOW_LED, LOW);
      digitalWrite(RED_LED, HIGH);
      digitalWrite(BUZZER_PIN, HIGH);

      if (currentPwmDuty < 255) {
        currentPwmDuty = (currentPwmDuty + 25 > 255) ? 255 : currentPwmDuty + 25;
        ledcWrite(PUMP_PWM_CHANNEL, currentPwmDuty);
        if (FAN_PIN != -1) ledcWrite(FAN_PWM_CHANNEL, currentPwmDuty);
      }

      if (!smsSentForBreach) {
        char msg[160];
        snprintf(msg, sizeof(msg), "ALARM BREACH!\r\nT:%.1fC H:%.1f%% VOC:%d",
                 filteredTemp, filteredHum, (int)filteredVoc);
        if (xQueueSend(gsmQueue, &msg, 0) == pdTRUE) smsSentForBreach = true;
      }

    } else {
      digitalWrite(GREEN_LED, HIGH);
      digitalWrite(YELLOW_LED, LOW);
      digitalWrite(RED_LED, LOW);
      digitalWrite(BUZZER_PIN, LOW);
      stopActuators();
      smsSentForBreach = false;
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void setup() {
  Serial.begin(115200);

  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }

  checkWakeupReason();
  loadSubscribers();

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

  xSystemEvents = xEventGroupCreate();
  xPowerSleepSemaphore = xSemaphoreCreateBinary();
  xActivityMutex = xSemaphoreCreateMutex();
  xSpiMutex = xSemaphoreCreateMutex();  
  
  gsmQueue = xQueueCreate(3, sizeof(char[160]));
  systemCmdQueue = xQueueCreate(10, sizeof(SystemCommandType));

  setSystemModeAtomic(MODE_INSPECT_BIT);

  dht.begin();
  initTinyML();

  // Create Hardware FreeRTOS Tasks
  xTaskCreatePinnedToCore(vTaskPowerManagement, "PowerTask",    3072,  NULL, 3, NULL, 0);
  xTaskCreatePinnedToCore(vTaskButtonHandler,    "ButtonTask",   2048,  NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(vTaskSystemEngine,     "SystemEngine", 10240, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(vTaskLoRaEngine,       "LoRaTask",     8192,  NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(vTaskGSM,              "GSM_Task",      4096,  NULL, 1, NULL, 1);
}

void loop() {
  vTaskDelete(NULL);
}