#include <Arduino.h>
#include <DHT.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/event_groups.h>
#include <driver/rtc_io.h>
#include "pins.h"

// --- TinyML TensorFlow Lite Headers & Model ---
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "grain_model.h" // Ensures g_grain_model or g_model_data array definition is present

// Fallback definition if model name differs in grain_model.h
#ifndef g_model_data
#ifdef g_grain_model
#define g_model_data g_grain_model
#endif
#endif

// --- PWM Configuration for Inductive Load Soft-Start ---
#define PUMP_PWM_CHANNEL  0
#define FAN_PWM_CHANNEL   1
#define PWM_FREQ          1000  // 1 kHz
#define PWM_RES           8     // 8-bit resolution (0-255)

// --- Event Group Bit Masks ---
#define MODE_INSPECT_BIT     (1UL << 0UL)  // 0x01
#define MODE_GUARDIAN_BIT    (1UL << 1UL)  // 0x02
#define THRESHOLD_BREACH_BIT (1UL << 2UL)  // 0x04

#define ALL_MODES_MASK       (MODE_INSPECT_BIT | MODE_GUARDIAN_BIT)

// --- Power & Sleep Timing Configurations ---
#define AWAKE_INTERVAL_SEC  (2 * 60)       // Active window (2 minutes)
#define SLEEP_DURATION_SEC (3 * 60)       // Deep sleep duration (3 minutes)
#define SLEEP_TIMER_US     (SLEEP_DURATION_SEC * 1000000ULL)

// --- TinyML Allocation Globals ---
namespace {
  tflite::ErrorReporter* errorReporter = nullptr;
  const tflite::Model* model = nullptr;
  tflite::MicroInterpreter* interpreter = nullptr;
  TfLiteTensor* input = nullptr;
  TfLiteTensor* output = nullptr;

  constexpr int kTensorArenaSize = 8 * 1024; // 8KB arena in internal RAM
  uint8_t tensorArena[kTensorArenaSize];
}

// --- Structs & Globals ---
struct SensorData {
    float temp;
    float humidity;
    int voc;
};

DHT dht(DHT_PIN, DHT_TYPE);
LiquidCrystal_I2C lcd(0x27, 16, 2); 
HardwareSerial gsmSerial(2); 
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// RTOS Handles
EventGroupHandle_t xSystemEvents = NULL;
QueueHandle_t gsmQueue = NULL;
SemaphoreHandle_t xPowerSleepSemaphore = NULL;
SemaphoreHandle_t xActivityMutex = NULL;
TaskHandle_t xSystemEngineTaskHandle = NULL;

SensorData latestMetrics = {-1.0f, -1.0f, 0};
bool smsSentForBreach = false;
uint8_t currentPwmDuty = 0; 
TickType_t xLastActivityTime = 0;

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

// --- Atomic State Controller ---
void setSystemModeAtomic(EventBits_t targetModeBit) {
    if (xSystemEvents == NULL) return;

    // Use standard task-level API calls without ISR primitives
    xEventGroupClearBits(xSystemEvents, ALL_MODES_MASK);
    xEventGroupSetBits(xSystemEvents, targetModeBit);

    resetInactivityTimer();
}

// --- TinyML Initialization ---
void initTinyML() {
    static tflite::MicroErrorReporter microErrorReporter;
    errorReporter = &microErrorReporter;

    model = tflite::GetModel(grain_model);
    if (model->version() != TFLITE_SCHEMA_VERSION) {
        TF_LITE_REPORT_ERROR(errorReporter, "Error: TinyML model schema mismatch!");
        return;
    }

    static tflite::AllOpsResolver resolver;
    static tflite::MicroInterpreter static_interpreter(
        model, resolver, tensorArena, kTensorArenaSize, errorReporter);
    interpreter = &static_interpreter;

    if (interpreter->AllocateTensors() != kTfLiteOk) {
        TF_LITE_REPORT_ERROR(errorReporter, "Error: TinyML AllocateTensors() failed!");
        return;
    }

    input = interpreter->input(0);
    output = interpreter->output(0);
    Serial.println("TinyML Interpreter initialized successfully.");
}

// --- Interrupt Service Routines ---
void IRAM_ATTR isrInspectButton() { 
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (xSystemEngineTaskHandle != NULL) {
        vTaskNotifyGiveFromISR(xSystemEngineTaskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void IRAM_ATTR isrModeButton() { 
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (xSystemEngineTaskHandle != NULL) {
        xTaskNotifyFromISR(xSystemEngineTaskHandle, 0x02, eSetBits, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void IRAM_ATTR isrPowerButton() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (xPowerSleepSemaphore != NULL) {
        xSemaphoreGiveFromISR(xPowerSleepSemaphore, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

// --- Non-Blocking Actuator Helper ---
void stopActuators() {
    currentPwmDuty = 0;
    ledcWrite(PUMP_PWM_CHANNEL, 0);
    if (FAN_PIN != -1) {
        ledcWrite(FAN_PWM_CHANNEL, 0);
    }
}

// --- Deep Sleep Helper ---
void triggerDeepSleep() {
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(YELLOW_LED, LOW);
    digitalWrite(RED_LED, LOW);
    digitalWrite(BUZZER_PIN, LOW);
    stopActuators();

    lcd.noBacklight();
    lcd.clear();

    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
    rtc_gpio_init((gpio_num_t)POWER_BUTTON);
    rtc_gpio_set_direction((gpio_num_t)POWER_BUTTON, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pullup_en((gpio_num_t)POWER_BUTTON);
    rtc_gpio_pulldown_dis((gpio_num_t)POWER_BUTTON);
    gpio_hold_en((gpio_num_t)POWER_BUTTON);

    esp_sleep_enable_ext0_wakeup((gpio_num_t)POWER_BUTTON, 0);
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
            vTaskDelay(pdMS_TO_TICKS(50));

            if (digitalRead(POWER_BUTTON) == LOW) {
                while (digitalRead(POWER_BUTTON) == LOW) {
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
                vTaskDelay(pdMS_TO_TICKS(200));
            }

            Serial.println("Manual Power-Down requested.");
            triggerDeepSleep();
        } 

        TickType_t xCurrentTicks = xTaskGetTickCount();
        TickType_t xElapsedTicks = xCurrentTicks - getLastActivityTime();

        if (xElapsedTicks >= xAwakeTimeoutTicks) {
            EventBits_t currentBits = xEventGroupGetBits(xSystemEvents);
            bool isBreached = (currentBits & THRESHOLD_BREACH_BIT) != 0;

            if (!isBreached) {
                Serial.printf("Awake interval (%d sec) expired with no breach. Entering deep sleep...\n", AWAKE_INTERVAL_SEC);
                triggerDeepSleep();
            } else {
                Serial.println("Breach detected! Deferring auto-sleep cycle.");
                resetInactivityTimer();
            }
        }
    }
}

// --- MQTT Callback ---
void mqttCallback(char* topic, byte* payload, unsigned int length) {
    resetInactivityTimer();

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, payload, length);
    if (!err) {
        if (doc.containsKey("mode")) {
            const char* modeStr = doc["mode"];
            if (strcmp(modeStr, "INSPECT") == 0)   setSystemModeAtomic(MODE_INSPECT_BIT);
            if (strcmp(modeStr, "GUARDIAN") == 0)  setSystemModeAtomic(MODE_GUARDIAN_BIT);
        }
        if (doc.containsKey("power") && strcmp(doc["power"], "OFF") == 0) {
            if (xPowerSleepSemaphore != NULL) {
                xSemaphoreGive(xPowerSleepSemaphore);
            }
        }
    }
}

// --- Task: MQTT Sync ---
void vTaskMQTT(void *pvParameters) {
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    mqttClient.setServer(MQTT_BROKER_IP, MQTT_PORT);
    mqttClient.setCallback(mqttCallback);

    for (;;) {
        if (WiFi.status() == WL_CONNECTED) {
            if (!mqttClient.connected()) {
                if (mqttClient.connect("ESP32_Guardian_Gateway")) {
                    mqttClient.subscribe("guardian/control");
                }
            } else {
                mqttClient.loop();

                EventBits_t currentBits = xEventGroupGetBits(xSystemEvents);
                bool isGuardian = (currentBits & MODE_GUARDIAN_BIT) != 0;
                bool isBreached = (currentBits & THRESHOLD_BREACH_BIT) != 0;

                JsonDocument doc;
                doc["mode"]        = isGuardian ? "GUARDIAN" : "INSPECT";
                doc["temperature"] = (latestMetrics.temp < 0.0f) ? 0.0 : latestMetrics.temp;
                doc["humidity"]    = (latestMetrics.humidity < 0.0f) ? 0.0 : latestMetrics.humidity;
                doc["voc"]         = latestMetrics.voc;
                doc["breach"]      = isBreached;

                bool actuatorActive = (isGuardian && isBreached);
                doc["actuators"]["buzzer"] = actuatorActive;
                doc["actuators"]["pump"]   = actuatorActive;
                doc["actuators"]["fan"]    = (FAN_PIN != -1) ? actuatorActive : false;

                char buffer[300];
                serializeJson(doc, buffer);
                mqttClient.publish("guardian/telemetry", buffer);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
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
    gsmSerial.println("AT+CNMI=2,2,0,0,0");
    vTaskDelay(pdMS_TO_TICKS(500));

    char alertMessage[160];

    for (;;) {
        if (xQueueReceive(gsmQueue, &alertMessage, pdMS_TO_TICKS(100)) == pdTRUE) {
            gsmSerial.println("AT+CMGF=1");
            vTaskDelay(pdMS_TO_TICKS(200));
            gsmSerial.printf("AT+CMGS=\"%s\"\r\n", TARGET_PHONE_NUM);
            vTaskDelay(pdMS_TO_TICKS(200));
            gsmSerial.print(alertMessage);
            gsmSerial.write(26);
            vTaskDelay(pdMS_TO_TICKS(3000));
        }

        if (gsmSerial.available()) {
            String incoming = gsmSerial.readString();
            incoming.trim();

            int bodyIdx = incoming.lastIndexOf("\r\n");
            String command = (bodyIdx != -1) ? incoming.substring(bodyIdx) : incoming;
            command.trim();

            if (command.equals("1")) {
                setSystemModeAtomic(MODE_INSPECT_BIT);
            } 
            else if (command.equals("2")) {
                setSystemModeAtomic(MODE_GUARDIAN_BIT);
            }
            else if (command.equals("3")) {
                if (xPowerSleepSemaphore != NULL) {
                    xSemaphoreGive(xPowerSleepSemaphore);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// --- Central Engine with Integrated TinyML Anomaly Detection ---
void vTaskSystemEngine(void *pvParameters) {
    TickType_t xLastDhtReadTime = xTaskGetTickCount();
    const TickType_t xDhtInterval = pdMS_TO_TICKS(2000);

    uint32_t ulNotificationValue = 0;

    for (;;) {
        // 1. Process deferred button interrupts
        if (xTaskNotifyWait(0x00, ULONG_MAX, &ulNotificationValue, 0) == pdTRUE) {
            if (ulNotificationValue & 0x02) {
                setSystemModeAtomic(MODE_GUARDIAN_BIT);
            } else {
                setSystemModeAtomic(MODE_INSPECT_BIT);
            }
        }

        // 2. Non-blocking periodic DHT reading
        TickType_t xNow = xTaskGetTickCount();
        if ((xNow - xLastDhtReadTime) >= xDhtInterval) {
            xLastDhtReadTime = xNow;

            float rawTemp = dht.readTemperature();
            float rawHum  = dht.readHumidity();
            bool dhtError = isnan(rawTemp) || isnan(rawHum);

            latestMetrics.temp = dhtError ? -1.0f : rawTemp;
            latestMetrics.humidity = dhtError ? -1.0f : rawHum;
        }

        // 3. Read MQ-2 Sensor
        int rawVoc = analogRead(MQ2_PIN);
        bool mq2Error = (rawVoc < 0 || rawVoc > 4095);
        latestMetrics.voc = mq2Error ? 0 : rawVoc;

        // 4. TinyML Anomaly Inference
        bool breachDetected = false;
        bool sensorsValid = (latestMetrics.temp >= 0.0f) && (latestMetrics.humidity >= 0.0f) && !mq2Error;

        if (sensorsValid && interpreter != nullptr) {
            input->data.f[0] = latestMetrics.temp / 100.0f;
            input->data.f[1] = latestMetrics.humidity / 100.0f;
            input->data.f[2] = (float)latestMetrics.voc / 4095.0f;

            if (interpreter->Invoke() == kTfLiteOk) {
                float anomalyProbability = output->data.f[1];
                if (anomalyProbability > 0.75f) {
                    breachDetected = true;
                }
            } else {
                breachDetected = (latestMetrics.temp > TEMP_THRESHOLD) || 
                                 (latestMetrics.humidity > HUM_THRESHOLD) || 
                                 (latestMetrics.voc > VOC_THRESHOLD);
            }
        } else if (sensorsValid) {
            breachDetected = (latestMetrics.temp > TEMP_THRESHOLD) || 
                             (latestMetrics.humidity > HUM_THRESHOLD) || 
                             (latestMetrics.voc > VOC_THRESHOLD);
        }

        if (breachDetected) {
            xEventGroupSetBits(xSystemEvents, THRESHOLD_BREACH_BIT);
        } else {
            xEventGroupClearBits(xSystemEvents, THRESHOLD_BREACH_BIT);
        }

        // 5. Query system state
        EventBits_t currentBits = xEventGroupWaitBits(
            xSystemEvents,
            ALL_MODES_MASK | THRESHOLD_BREACH_BIT,
            pdFALSE,             
            pdFALSE,             
            pdMS_TO_TICKS(200)   
        );

        bool isGuardian = (currentBits & MODE_GUARDIAN_BIT) != 0;
        bool isBreached = (currentBits & THRESHOLD_BREACH_BIT) != 0;

        // 6. Actuation
        if (!isGuardian) { // INSPECT MODE
            digitalWrite(GREEN_LED, HIGH);
            digitalWrite(YELLOW_LED, LOW);
            digitalWrite(RED_LED, LOW);
            digitalWrite(BUZZER_PIN, LOW);
            stopActuators();
            smsSentForBreach = false;
        } 
        else { // GUARDIAN MODE
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
                    snprintf(msg, sizeof(msg), 
                        "ALARM BREACH!\r\nT:%.1fC H:%.1f%% VOC:%d\r\n"
                        "Reply Option:\r\n"
                        "1-Inspect\r\n"
                        "2-Guardian\r\n"
                        "3-Power OFF",
                        latestMetrics.temp, latestMetrics.humidity, latestMetrics.voc);
                    if (xQueueSend(gsmQueue, &msg, 0) == pdTRUE) {
                        smsSentForBreach = true;
                    }
                }
            } else {
                digitalWrite(GREEN_LED, LOW);
                digitalWrite(YELLOW_LED, HIGH);
                digitalWrite(RED_LED, LOW);
                digitalWrite(BUZZER_PIN, LOW);
                stopActuators();
                smsSentForBreach = false;
            }
        }
    }
}

// --- Display Task ---
void vTaskLCD(void *pvParameters) {
    for (;;) {
        EventBits_t currentBits = xEventGroupGetBits(xSystemEvents);
        bool isGuardian = (currentBits & MODE_GUARDIAN_BIT) != 0;
        bool isBreached = (currentBits & THRESHOLD_BREACH_BIT) != 0;

        bool dhtError = (latestMetrics.temp < 0.0f) || (latestMetrics.humidity < 0.0f);
        bool mq2Error = (latestMetrics.voc < 0 || latestMetrics.voc > 4095);

        // Row 1: Mode & Climate
        lcd.setCursor(0, 0);
        if (dhtError) {
            lcd.print("MODE:ERR DHT ERR ");
        } else {
            lcd.printf("%-4s T:%2dC H:%2d%% ", 
                       isGuardian ? "GARD" : "INSP", 
                       (int)latestMetrics.temp, 
                       (int)latestMetrics.humidity);
        }

        // Row 2: Air Quality & Status
        lcd.setCursor(0, 1);
        if (mq2Error) {
            lcd.print("MQ2 SENSOR ERR  ");
        } else {
            lcd.printf("VOC:%-4d ST:%-5s", 
                       latestMetrics.voc, 
                       isBreached ? "ALERT" : "OK   ");
        }

        vTaskDelay(pdMS_TO_TICKS(1500));
    }
}

// --- System Initialization ---
void setup() {
    Serial.begin(115200);

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

    pinMode(INSPECT_BUTTON, INPUT_PULLUP);
    pinMode(MODE_BUTTON, INPUT_PULLUP);
    pinMode(POWER_BUTTON, INPUT_PULLUP);

    xSystemEvents = xEventGroupCreate();
    xPowerSleepSemaphore = xSemaphoreCreateBinary();
    xActivityMutex = xSemaphoreCreateMutex();

    setSystemModeAtomic(MODE_INSPECT_BIT);

    attachInterrupt(digitalPinToInterrupt(INSPECT_BUTTON), isrInspectButton, FALLING);
    attachInterrupt(digitalPinToInterrupt(MODE_BUTTON), isrModeButton, FALLING);
    attachInterrupt(digitalPinToInterrupt(POWER_BUTTON), isrPowerButton, FALLING);

    Wire.begin(SDA_PIN, SCL_PIN);
    lcd.init();
    lcd.backlight();
    dht.begin();

    // Initialize TensorFlow Lite Micro Engine
    initTinyML();

    gsmQueue = xQueueCreate(3, sizeof(char[160]));

    xTaskCreatePinnedToCore(vTaskPowerManagement, "PowerTask", 2048, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(vTaskSystemEngine, "SystemEngine", 6144, NULL, 2, &xSystemEngineTaskHandle, 0);
    xTaskCreatePinnedToCore(vTaskGSM, "GSM_Task", 3072, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(vTaskMQTT, "MQTT_Task", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(vTaskLCD, "LCD_Task", 2048, NULL, 1, NULL, 1);
}

void loop() {
    vTaskDelete(NULL);
}