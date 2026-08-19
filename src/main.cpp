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
#include <WiFi.h>

// --- TinyML TensorFlow Lite Headers & Model ---
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
#define PUMP_PWM_CHANNEL    0
#define FAN_PWM_CHANNEL     1
#define PWM_FREQ            1000
#define PWM_RES             8

#define LORA_FREQ           868E6  // Match Ground Station frequency

#define MODE_INSPECT_BIT     (1UL << 0UL)
#define MODE_GUARDIAN_BIT    (1UL << 1UL)
#define THRESHOLD_BREACH_BIT (1UL << 2UL)
#define ALL_MODES_MASK       (MODE_INSPECT_BIT | MODE_GUARDIAN_BIT)

#define AWAKE_INTERVAL_SEC  (2 * 60)
#define SLEEP_DURATION_SEC  (3 * 60)
#define SLEEP_TIMER_US      (SLEEP_DURATION_SEC * 1000000ULL)

#define MAX_SUBSCRIBERS     5
#define PHONE_LEN           16

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

// --- Data Models & Commands ---
enum SystemCommandType {
  CMD_SET_MODE_INSPECT,
  CMD_SET_MODE_GUARDIAN,
  CMD_TRIGGER_SLEEP
};

struct SensorData {
  float temp;
  float humidity;
  int voc;
};

// --- Global Objects & Handles ---
DHT dht(DHT_PIN, DHT_TYPE);
HardwareSerial gsmSerial(2);
Preferences preferences;

EventGroupHandle_t xSystemEvents = NULL;
QueueHandle_t gsmQueue = NULL;
QueueHandle_t systemCmdQueue = NULL;
SemaphoreHandle_t xPowerSleepSemaphore = NULL;
SemaphoreHandle_t xActivityMutex = NULL;
TaskHandle_t xSystemEngineTaskHandle = NULL;

SensorData latestMetrics = {-1.0f, -1.0f, 0};
bool smsSentForBreach = false;
uint8_t currentPwmDuty = 0;
TickType_t xLastActivityTime = 0;

// Dynamic GSM Subscribers List
char subscribers[MAX_SUBSCRIBERS][PHONE_LEN];
uint8_t subscriberCount = 0;

// --- Helper Functions: Persistent GSM Subscriber Storage ---
void loadSubscribers() {
  preferences.begin("gsm_subs", true);
  subscriberCount = preferences.getUChar("count", 0);
  for (uint8_t i = 0; i < subscriberCount; i++) {
    String key = "sub" + String(i);
    String num = preferences.getString(key.c_str(), "");
    strncpy(subscribers[i], num.c_str(), PHONE_LEN - 1);
  }
  preferences.end();
}

bool addSubscriber(const char* phone) {
  if (subscriberCount >= MAX_SUBSCRIBERS) return false;
  for (uint8_t i = 0; i < subscriberCount; i++) {
    if (strcmp(subscribers[i], phone) == 0) return true;
  }
  strncpy(subscribers[subscriberCount], phone, PHONE_LEN - 1);
  subscriberCount++;

  preferences.begin("gsm_subs", false);
  preferences.putUChar("count", subscriberCount);
  String key = "sub" + String(subscriberCount - 1);
  preferences.putString(key.c_str(), phone);
  preferences.end();
  return true;
}

bool removeSubscriber(const char* phone) {
  int foundIdx = -1;
  for (uint8_t i = 0; i < subscriberCount; i++) {
    if (strcmp(subscribers[i], phone) == 0) {
      foundIdx = i;
      break;
    }
  }
  if (foundIdx == -1) return false;

  for (uint8_t i = foundIdx; i < subscriberCount - 1; i++) {
    strcpy(subscribers[i], subscribers[i + 1]);
  }
  subscriberCount--;

  preferences.begin("gsm_subs", false);
  preferences.clear();
  preferences.putUChar("count", subscriberCount);
  for (uint8_t i = 0; i < subscriberCount; i++) {
    String key = "sub" + String(i);
    preferences.putString(key.c_str(), subscribers[i]);
  }
  preferences.end();
  return true;
}

// --- Thread-Safe Inactivity Tracking ---
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

// --- TinyML Initialization ---
void initTinyML() {
  static tflite::MicroErrorReporter microErrorReporter;
  errorReporter = &microErrorReporter;
  model = tflite::GetModel(g_model);
  if (model->version() != TFLITE_SCHEMA_VERSION) return;

  static tflite::AllOpsResolver resolver;
  static tflite::MicroInterpreter static_interpreter(
      model, resolver, tensorArena, kTensorArenaSize, errorReporter);
  interpreter = &static_interpreter;

  if (interpreter->AllocateTensors() != kTfLiteOk) return;
  input = interpreter->input(0);
  output = interpreter->output(0);
}

// --- Actuator Management & Sleep Procedures ---
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

  esp_sleep_enable_timer_wakeup(SLEEP_TIMER_US);
  Serial.flush();
  esp_deep_sleep_start();
}

// --- Task: Power Management ---
void vTaskPowerManagement(void *pvParameters) {
  resetInactivityTimer();
  const TickType_t xAwakeTimeoutTicks = pdMS_TO_TICKS(AWAKE_INTERVAL_SEC * 1000);

  for (;;) {
    // Process explicit sleep command triggered remotely via LoRa
    if (xSemaphoreTake(xPowerSleepSemaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
      triggerDeepSleep();
    }

    // Timer-based inactive auto-sleep
    TickType_t xCurrentTicks = xTaskGetTickCount();
    if ((xCurrentTicks - getLastActivityTime()) >= xAwakeTimeoutTicks) {
      EventBits_t currentBits = xEventGroupGetBits(xSystemEvents);
      if ((currentBits & THRESHOLD_BREACH_BIT) == 0) {
        triggerDeepSleep();
      } else {
        resetInactivityTimer();
      }
    }
  }
}

// --- Task: Bi-Directional LoRa Engine ---
void vTaskLoRaEngine(void *pvParameters) {
  SPI.begin(LORA_SCK_PIN, LORA_MISO_PIN, LORA_MOSI_PIN, LORA_CS_PIN);
  LoRa.setPins(LORA_CS_PIN, LORA_RST_PIN, LORA_DIO0_PIN);

  if (!LoRa.begin(LORA_FREQ)) {
    vTaskDelete(NULL);
  }

  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.enableCrc();

  TickType_t xLastTx = xTaskGetTickCount();

  for (;;) {
    // Receive commands over LoRa
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
      resetInactivityTimer();
      String payload = "";
      while (LoRa.available()) payload += (char)LoRa.read();

      StaticJsonDocument<128> doc;
      DeserializationError err = deserializeJson(doc, payload);

      if (!err && doc.containsKey("cmd")) {
        const char* cmd = doc["cmd"];
        const char* val = doc["val"];

        if (strcmp(cmd, "SET_MODE") == 0) {
          SystemCommandType sysCmd = (strcmp(val, "GUARDIAN") == 0) ? CMD_SET_MODE_GUARDIAN : CMD_SET_MODE_INSPECT;
          xQueueSend(systemCmdQueue, &sysCmd, 0);
        } else if (strcmp(cmd, "POWER") == 0 && strcmp(val, "TOGGLE") == 0) {
          SystemCommandType sysCmd = CMD_TRIGGER_SLEEP;
          xQueueSend(systemCmdQueue, &sysCmd, 0);
        }
      }
    }

    // Transmit Telemetry every 2 seconds
    if ((xTaskGetTickCount() - xLastTx) >= pdMS_TO_TICKS(2000)) {
      xLastTx = xTaskGetTickCount();

      EventBits_t currentBits = xEventGroupGetBits(xSystemEvents);
      bool isGuardian = (currentBits & MODE_GUARDIAN_BIT) != 0;
      bool isBreached = (currentBits & THRESHOLD_BREACH_BIT) != 0;

      StaticJsonDocument<256> doc;
      doc["temp"] = latestMetrics.temp;
      doc["hum"] = latestMetrics.humidity;
      doc["voc"] = latestMetrics.voc;
      doc["mode"] = isGuardian ? "GUARDIAN" : "MONITOR";
      doc["breach"] = isBreached;

      char buffer[256];
      size_t len = serializeJson(doc, buffer);

      LoRa.beginPacket();
      LoRa.write((const uint8_t*)buffer, len);
      LoRa.endPacket();
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// --- Task: GSM Engine ---
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

    if (gsmSerial.available()) {
      String incoming = gsmSerial.readString();
      incoming.trim();

      if (incoming.indexOf("1") != -1) {
        SystemCommandType cmd = CMD_SET_MODE_INSPECT;
        xQueueSend(systemCmdQueue, &cmd, 0);
      } else if (incoming.indexOf("2") != -1) {
        SystemCommandType cmd = CMD_SET_MODE_GUARDIAN;
        xQueueSend(systemCmdQueue, &cmd, 0);
      } else if (incoming.indexOf("ADD:") != -1) {
        int idx = incoming.indexOf("ADD:");
        String phone = incoming.substring(idx + 4);
        phone.trim();
        addSubscriber(phone.c_str());
      } else if (incoming.indexOf("REMOVE:") != -1) {
        int idx = incoming.indexOf("REMOVE:");
        String phone = incoming.substring(idx + 7);
        phone.trim();
        removeSubscriber(phone.c_str());
      }
    }
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

// --- Task: Central System Engine ---
void vTaskSystemEngine(void *pvParameters) {
  TickType_t xLastDhtReadTime = xTaskGetTickCount();
  const TickType_t xDhtInterval = pdMS_TO_TICKS(2000);
  SystemCommandType inboundCmd;

  for (;;) {
    if (xQueueReceive(systemCmdQueue, &inboundCmd, 0) == pdTRUE) {
      resetInactivityTimer();
      switch (inboundCmd) {
        case CMD_SET_MODE_INSPECT:
          setSystemModeAtomic(MODE_INSPECT_BIT);
          break;
        case CMD_SET_MODE_GUARDIAN:
          setSystemModeAtomic(MODE_GUARDIAN_BIT);
          break;
        case CMD_TRIGGER_SLEEP:
          if (xPowerSleepSemaphore != NULL) xSemaphoreGive(xPowerSleepSemaphore);
          break;
      }
    }

    TickType_t xNow = xTaskGetTickCount();
    if ((xNow - xLastDhtReadTime) >= xDhtInterval) {
      xLastDhtReadTime = xNow;
      float rawTemp = dht.readTemperature();
      float rawHum  = dht.readHumidity();
      latestMetrics.temp = (isnan(rawTemp)) ? -1.0f : rawTemp;
      latestMetrics.humidity = (isnan(rawHum)) ? -1.0f : rawHum;
    }

    int rawVoc = analogRead(MQ2_PIN);
    latestMetrics.voc = (rawVoc < 0 || rawVoc > 4095) ? 0 : rawVoc;

    bool breachDetected = false;
    bool sensorsValid = (latestMetrics.temp >= 0.0f) && (latestMetrics.humidity >= 0.0f);

    if (sensorsValid && interpreter != nullptr) {
      input->data.f[0] = latestMetrics.temp / 100.0f;
      input->data.f[1] = latestMetrics.humidity / 100.0f;
      input->data.f[2] = (float)latestMetrics.voc / 4095.0f;

      if (interpreter->Invoke() == kTfLiteOk) {
        if (output->data.f[1] > 0.75f) breachDetected = true;
      } else {
        breachDetected = (latestMetrics.temp > TEMP_THRESHOLD) || 
                         (latestMetrics.humidity > HUM_THRESHOLD) || 
                         (latestMetrics.voc > VOC_THRESHOLD);
      }
    }

    if (breachDetected) xEventGroupSetBits(xSystemEvents, THRESHOLD_BREACH_BIT);
    else xEventGroupClearBits(xSystemEvents, THRESHOLD_BREACH_BIT);

    EventBits_t currentBits = xEventGroupGetBits(xSystemEvents);
    bool isGuardian = (currentBits & MODE_GUARDIAN_BIT) != 0;
    bool isBreached = (currentBits & THRESHOLD_BREACH_BIT) != 0;

    if (!isGuardian) {
      digitalWrite(GREEN_LED, HIGH);
      digitalWrite(YELLOW_LED, LOW);
      digitalWrite(RED_LED, LOW);
      digitalWrite(BUZZER_PIN, LOW);
      stopActuators();
      smsSentForBreach = false;
    } else {
      if (isBreached) {
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
                   latestMetrics.temp, latestMetrics.humidity, latestMetrics.voc);
          if (xQueueSend(gsmQueue, &msg, 0) == pdTRUE) smsSentForBreach = true;
        }
      } else {
        digitalWrite(GREEN_LED, LOW);
        digitalWrite(YELLOW_LED, HIGH);
        digitalWrite(RED_LED, LOW);
        digitalWrite(BUZZER_PIN, LOW);
        stopActuators();
        smsSentForBreach = false;

          if (currentPwmDuty < 255) {
          currentPwmDuty = (currentPwmDuty + 25 > 255) ? 255 : currentPwmDuty + 25;
          ledcWrite(PUMP_PWM_CHANNEL, currentPwmDuty);
          if (FAN_PIN != -1) ledcWrite(FAN_PWM_CHANNEL, currentPwmDuty);
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// --- Setup ---
void setup() {
  Serial.begin(115200);

  loadSubscribers();

  pinMode(GREEN_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  ledcSetup(PUMP_PWM_CHANNEL, PWM_FREQ, PWM_RES);
  ledcAttachPin(PUMP_PIN, PUMP_PWM_CHANNEL);
  if (FAN_PIN != -1) {
    ledcSetup(FAN_PWM_CHANNEL, PWM_FREQ, PWM_RES);
    ledcAttachPin(FAN_PIN, FAN_PWM_CHANNEL);
  }

  xSystemEvents = xEventGroupCreate();
  xPowerSleepSemaphore = xSemaphoreCreateBinary();
  xActivityMutex = xSemaphoreCreateMutex();
  gsmQueue = xQueueCreate(3, sizeof(char[160]));
  systemCmdQueue = xQueueCreate(10, sizeof(SystemCommandType));

  setSystemModeAtomic(MODE_INSPECT_BIT);

  dht.begin();
  initTinyML();

  // Task Pinning
  xTaskCreatePinnedToCore(vTaskPowerManagement, "PowerTask", 2048, NULL, 3, NULL, 0);
  xTaskCreatePinnedToCore(vTaskSystemEngine, "SystemEngine", 6144, NULL, 2, &xSystemEngineTaskHandle, 0);
  xTaskCreatePinnedToCore(vTaskLoRaEngine, "LoRaTask", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(vTaskGSM, "GSM_Task", 3072, NULL, 1, NULL, 1);
}

void loop() {
  vTaskDelete(NULL);
}