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
#include "protocol.h"
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


#define MODE_INSPECT_BIT     (1UL << 0UL)
#define MODE_OVERRIDE_BIT    (1UL << 1UL)
#define THRESHOLD_BREACH_BIT (1UL << 2UL)
#define ALL_MODES_MASK       (MODE_INSPECT_BIT | MODE_OVERRIDE_BIT)

#define AWAKE_INTERVAL_SEC  (2 * 60)
#define SLEEP_DURATION_SEC  (3 * 60)
#define SLEEP_TIMER_US      (SLEEP_DURATION_SEC * 1000000ULL)

#define MAX_SUBSCRIBERS     5
#define PHONE_LEN           16

// Note: TEMP_THRESHOLD, HUM_THRESHOLD, and VOC_THRESHOLD are imported from pins.h

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
  CMD_SET_MODE_OVERRIDE,
  CMD_TRIGGER_SLEEP
};

// The seq travels with the command so "ack" reports the sequence actually
// APPLIED, not merely the one received.
struct SystemCommand {
  SystemCommandType type;
  uint8_t seq;
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
volatile uint8_t lastAppliedSeq = SEQ_NONE;

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

// --- Task: Bi-Directional LoRa Engine ---
void vTaskLoRaEngine(void *pvParameters) {
  Serial.println("[LoRa] Initializing SPI & SX127x Module...");
  SPI.begin(LORA_SCK_PIN, LORA_MISO_PIN, LORA_MOSI_PIN, LORA_CS_PIN);
  LoRa.setPins(LORA_CS_PIN, LORA_RST_PIN, LORA_DIO0_PIN);

  if (!LoRa.begin(LORA_FREQ_HZ)) {
    Serial.println("[LoRa ERROR] Initialization Failed! Check wiring & power.");
    vTaskDelete(NULL);
  }

  LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
  LoRa.setSignalBandwidth(LORA_BANDWIDTH_HZ);
  LoRa.setCodingRate4(LORA_CODING_RATE);
  LoRa.enableCrc();

  Serial.println("[LoRa SUCCESS] Transceiver Ready @ 868MHz");

  TickType_t xLastTx = xTaskGetTickCount();

  for (;;) {
    // --- RECEIVE ENGINE ---
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
      resetInactivityTimer();
      String payload = "";
      while (LoRa.available()) payload += (char)LoRa.read();

      int rssi = LoRa.packetRssi();
      float snr = LoRa.packetSnr();

      Serial.println("\n------------------ LORA RX ------------------");
      Serial.printf("[LoRa RX] Bytes: %d | RSSI: %d dBm | SNR: %.2f dB\n", packetSize, rssi, snr);
      Serial.printf("[LoRa RX] Payload: %s\n", payload.c_str());

      JsonDocument doc;
      DeserializationError err = deserializeJson(doc, payload);

      if (err) {
        Serial.printf("[LoRa RX ERROR] JSON Parsing Failed: %s\n", err.c_str());
      } else {
        if (doc[K_CMD].is<const char*>()) {
          const char* cmd = doc[K_CMD];
          const char* val = doc[K_VAL] | "";
          uint8_t seq    = doc[K_SEQ] | SEQ_NONE;

          Serial.printf("[LoRa RX CMD] '%s' | '%s' | seq=%u\n", cmd, val, seq);

          SystemCommand sysCmd;
          sysCmd.seq = seq;
          bool recognised = true;

          if (strcmp(cmd, CMD_SET_MODE) == 0) {
            // Legacy spellings are still accepted so a half-flashed pair keeps
            // working during a rolling upgrade. Remove once both ends are on
            // protocol v2.
            if (strcmp(val, VAL_MANUAL) == 0 ||
                strcmp(val, "GUARDIAN") == 0 || strcmp(val, "OVERRIDE") == 0) {
              sysCmd.type = CMD_SET_MODE_OVERRIDE;
              Serial.println("[LoRa ACTION] -> MANUAL override");
            } else if (strcmp(val, VAL_AUTO) == 0 ||
                       strcmp(val, "MONITOR") == 0 || strcmp(val, "INSPECT") == 0) {
              sysCmd.type = CMD_SET_MODE_INSPECT;
              Serial.println("[LoRa ACTION] -> AUTO");
            } else {
              recognised = false;
              Serial.printf("[LoRa WARN] Unknown SET_MODE value: %s\n", val);
            }
          } else if (strcmp(cmd, CMD_POWER) == 0 &&
                     (strcmp(val, VAL_SLEEP) == 0 ||
                      strcmp(val, "TOGGLE") == 0 || strcmp(val, "OFF") == 0)) {
            sysCmd.type = CMD_TRIGGER_SLEEP;
            Serial.println("[LoRa ACTION] -> SLEEP");
          } else {
            recognised = false;
            Serial.printf("[LoRa WARN] Unhandled Command Key: %s\n", cmd);
          }

          if (recognised) xQueueSend(systemCmdQueue, &sysCmd, 0);
        } else {
          Serial.println("[LoRa RX WARN] Valid JSON, but missing 'cmd' string key");
        }
      }
      Serial.println("---------------------------------------------\n");
    }

    // --- TRANSMIT TELEMETRY ENGINE (Every 2s) ---
    if ((xTaskGetTickCount() - xLastTx) >= pdMS_TO_TICKS(2000)) {
      xLastTx = xTaskGetTickCount();

      EventBits_t currentBits = xEventGroupGetBits(xSystemEvents);
      bool isOverride = (currentBits & MODE_OVERRIDE_BIT) != 0;
      bool isBreached = (currentBits & THRESHOLD_BREACH_BIT) != 0;

      JsonDocument doc;
      doc[K_TEMP] = latestMetrics.temp;
      doc[K_HUM]  = latestMetrics.humidity;
      doc[K_VOC]  = latestMetrics.voc;

      // Two independent booleans rather than one overloaded "mode" string, so
      // neither the ground station nor the dashboard has to infer one fact from
      // the absence of another.
      doc[K_MANUAL] = isOverride;
      doc[K_BREACH] = isBreached;

      // Echo the last command actually applied, so the sender can confirm
      // delivery rather than guessing from a state change.
      doc[K_ACK] = lastAppliedSeq;

      char buffer[256];
      size_t len = serializeJson(doc, buffer);

      Serial.printf("[LoRa TX] Sending Telemetry (%d bytes): %s ... ", len, buffer);

      LoRa.beginPacket();
      LoRa.write((const uint8_t*)buffer, len);
      if (LoRa.endPacket()) {
        Serial.println("OK");
      } else {
        Serial.println("FAILED!");
      }
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
        SystemCommand cmd = { CMD_SET_MODE_INSPECT, SEQ_NONE };
        xQueueSend(systemCmdQueue, &cmd, 0);
      } else if (incoming.indexOf("2") != -1) {
        SystemCommand cmd = { CMD_SET_MODE_OVERRIDE, SEQ_NONE };
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
  SystemCommand inboundCmd;

  for (;;) {
    if (xQueueReceive(systemCmdQueue, &inboundCmd, 0) == pdTRUE) {
      resetInactivityTimer();
      switch (inboundCmd.type) {
        case CMD_SET_MODE_INSPECT:
          setSystemModeAtomic(MODE_INSPECT_BIT);
          break;
        case CMD_SET_MODE_OVERRIDE:
          setSystemModeAtomic(MODE_OVERRIDE_BIT);
          break;
        case CMD_TRIGGER_SLEEP:
          if (xPowerSleepSemaphore != NULL) xSemaphoreGive(xPowerSleepSemaphore);
          break;
      }
      // Recorded only once the command has actually been applied, so "ack"
      // never claims more than the node has done.
      if (inboundCmd.seq != SEQ_NONE) lastAppliedSeq = inboundCmd.seq;
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
      }
    } else {
      if (latestMetrics.voc > VOC_THRESHOLD || 
         (latestMetrics.temp > TEMP_THRESHOLD && latestMetrics.temp >= 0.0f) || 
         (latestMetrics.humidity > HUM_THRESHOLD && latestMetrics.humidity >= 0.0f)) {
        breachDetected = true;
      }
    }

    if (breachDetected) xEventGroupSetBits(xSystemEvents, THRESHOLD_BREACH_BIT);
    else xEventGroupClearBits(xSystemEvents, THRESHOLD_BREACH_BIT);

    EventBits_t currentBits = xEventGroupGetBits(xSystemEvents);
    bool isOverride = (currentBits & MODE_OVERRIDE_BIT) != 0;
    bool isBreached = (currentBits & THRESHOLD_BREACH_BIT) != 0;

    // --- State Machine Execution ---
    if (isOverride) {
      // MANUAL OVERRIDE MODE (Forces full system activation)
      digitalWrite(GREEN_LED, LOW);
      digitalWrite(YELLOW_LED, HIGH);
      digitalWrite(RED_LED, LOW);
      digitalWrite(BUZZER_PIN, HIGH);

      ledcWrite(PUMP_PWM_CHANNEL, 255);
      if (FAN_PIN != -1) ledcWrite(FAN_PWM_CHANNEL, 255);
      smsSentForBreach = false;

    } else if (isBreached) {
      // GUARDIAN BREACH MODE (Environment triggers alarm & actuation ramp up)
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
      // INSPECT / MONITOR MODE (Normal resting state)
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
  systemCmdQueue = xQueueCreate(10, sizeof(SystemCommand));

  setSystemModeAtomic(MODE_INSPECT_BIT);

  dht.begin();
  initTinyML();

  // Task Pinning
  xTaskCreatePinnedToCore(vTaskPowerManagement, "PowerTask", 2048, NULL, 3, NULL, 0);
  xTaskCreatePinnedToCore(vTaskSystemEngine, "SystemEngine", 6144, NULL, 2, &xSystemEngineTaskHandle, 0);
  xTaskCreatePinnedToCore(vTaskLoRaEngine, "LoRaTask", 8192, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(vTaskGSM, "GSM_Task", 3072, NULL, 1, NULL, 1);
}

void loop() {
  vTaskDelete(NULL);
}