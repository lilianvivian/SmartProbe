
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
#include "pins.h"
#include <driver/rtc_io.h>

// --- PWM Configuration for Inductive Load Soft-Start ---
#define PUMP_PWM_CHANNEL 0
#define FAN_PWM_CHANNEL   1
#define PWM_FREQ         1000  // 1 kHz
#define PWM_RES          8     // 8-bit resolution (0-255)

// --- Event Group Bit Masks ---
#define MODE_INSPECT_BIT     (1UL << 0UL)  // 0x01
#define MODE_GUARDIAN_BIT    (1UL << 1UL)  // 0x02
#define THRESHOLD_BREACH_BIT (1UL << 2UL)  // 0x04

#define ALL_MODES_MASK       (MODE_INSPECT_BIT | MODE_GUARDIAN_BIT)

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

// RTOS Sync Handles
EventGroupHandle_t xSystemEvents = NULL;
QueueHandle_t gsmQueue = NULL;
SemaphoreHandle_t xPowerSleepSemaphore = NULL;

SensorData latestMetrics = {0.0f, 0.0f, 0};
bool smsSentForBreach = false;
uint8_t currentPwmDuty = 0; // State variable for non-blocking soft-start

// --- Helper for Safe Bidirectional Task Transitions ---
void setSystemMode(EventBits_t modeBit) {
    xEventGroupClearBits(xSystemEvents, ALL_MODES_MASK);
    xEventGroupSetBits(xSystemEvents, modeBit);
}

// --- Interrupt Service Routines ---
volatile uint32_t lastButtonPress = 0;

void IRAM_ATTR isrInspectButton() { 
    uint32_t now = millis();
    if (now - lastButtonPress > 200) {
        lastButtonPress = now;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xEventGroupClearBitsFromISR(xSystemEvents, MODE_GUARDIAN_BIT);
        xEventGroupSetBitsFromISR(xSystemEvents, MODE_INSPECT_BIT, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void IRAM_ATTR isrModeButton() { 
    uint32_t now = millis();
    if (now - lastButtonPress > 200) {
        lastButtonPress = now;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xEventGroupClearBitsFromISR(xSystemEvents, MODE_INSPECT_BIT);
        xEventGroupSetBitsFromISR(xSystemEvents, MODE_GUARDIAN_BIT, &xHigherPriorityTaskWoken);
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

// --- Deferred Power Management Task ---
void vTaskPowerManagement(void *pvParameters) {
    for (;;) {
        if (xSemaphoreTake(xPowerSleepSemaphore, portMAX_DELAY) == pdTRUE) {
            vTaskDelay(pdMS_TO_TICKS(50));

            if(digitalRead(POWER_BUTTON) == HIGH) continue;

            while (digitalRead(POWER_BUTTON) == LOW) {
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            vTaskDelay(pdMS_TO_TICKS(200));

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
            Serial.flush();
            esp_deep_sleep_start();
        }
    }
}

// --- MQTT Callback ---
void mqttCallback(char* topic, byte* payload, unsigned int length) {
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, payload, length);
    if (!err) {
        if (doc.containsKey("mode")) {
            const char* modeStr = doc["mode"];
            if (strcmp(modeStr, "INSPECT") == 0)   setSystemMode(MODE_INSPECT_BIT);
            if (strcmp(modeStr, "GUARDIAN") == 0)  setSystemMode(MODE_GUARDIAN_BIT);
        }
        if (doc.containsKey("power") && strcmp(doc["power"], "OFF") == 0) {
            if (xPowerSleepSemaphore != NULL) {
                xSemaphoreGive(xPowerSleepSemaphore);
            }
        }
    }
}

// --- Task: MQTT Dashboard Sync ---
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
                doc["temperature"] = isnan(latestMetrics.temp) ? 0.0 : latestMetrics.temp;
                doc["humidity"]    = isnan(latestMetrics.humidity) ? 0.0 : latestMetrics.humidity;
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
    gsmSerial.begin(9600, SERIAL_8N1, GSM_RX_PIN, GSM_TX_PIN);
    vTaskDelay(pdMS_TO_TICKS(1000));

    gsmSerial.println("AT");
    vTaskDelay(pdMS_TO_TICKS(500));
    gsmSerial.println("AT+CMGF=1");
    vTaskDelay(pdMS_TO_TICKS(500));
    gsmSerial.println("AT+CNMI=2,2,0,0,0");
    vTaskDelay(pdMS_TO_TICKS(500));

    char alertMessage[64];

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
            if (incoming.indexOf("MODE=INSPECT") >= 0)  setSystemMode(MODE_INSPECT_BIT);
            if (incoming.indexOf("MODE=GUARDIAN") >= 0) setSystemMode(MODE_GUARDIAN_BIT);
            if (incoming.indexOf("POWER=OFF") >= 0) {
                if (xPowerSleepSemaphore != NULL) {
                    xSemaphoreGive(xPowerSleepSemaphore);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// --- Central Event-Driven Engine Task ---
void vTaskSystemEngine(void *pvParameters) {
    for (;;) {
        // 1. Read Sensors
        float rawTemp = dht.readTemperature();
        float rawHum  = dht.readHumidity();
        bool dhtError = isnan(rawTemp) || isnan(rawHum);

        latestMetrics.temp = dhtError ? 0.0f : rawTemp;
        latestMetrics.humidity = dhtError ? 0.0f : rawHum;

        int rawVoc = analogRead(MQ2_PIN);
        bool mq2Error = (rawVoc < 0 || rawVoc > 4095);
        latestMetrics.voc = mq2Error ? 0 : rawVoc;

        // 2. Evaluate and Set Breach Bit
        bool breachDetected = (!dhtError && (latestMetrics.temp > TEMP_THRESHOLD || latestMetrics.humidity > HUM_THRESHOLD)) ||
                             (!mq2Error && (latestMetrics.voc > VOC_THRESHOLD));

        if (breachDetected) {
            xEventGroupSetBits(xSystemEvents, THRESHOLD_BREACH_BIT);
        } else {
            xEventGroupClearBits(xSystemEvents, THRESHOLD_BREACH_BIT);
        }

        // 3. Atomically Wait and React to Event Group Bit States
        // Unblocks immediately when state bits update, or every 200ms periodically
        EventBits_t currentBits = xEventGroupWaitBits(
            xSystemEvents,
            ALL_MODES_MASK | THRESHOLD_BREACH_BIT,
            pdFALSE,             // Keep bits latched
            pdFALSE,             // React to ANY bit combination
            pdMS_TO_TICKS(200)   // Fast refresh period
        );

        bool isGuardian = (currentBits & MODE_GUARDIAN_BIT) != 0;
        bool isBreached = (currentBits & THRESHOLD_BREACH_BIT) != 0;

        // 4. State Actuation Engine
        if (!isGuardian) { // INSPECT MODE ACTIVE
            digitalWrite(GREEN_LED, HIGH);
            digitalWrite(YELLOW_LED, LOW);
            digitalWrite(RED_LED, LOW);
            digitalWrite(BUZZER_PIN, LOW);
            stopActuators();
            smsSentForBreach = false;
        } 
        else { // GUARDIAN MODE ACTIVE
            if (isBreached) {
                digitalWrite(GREEN_LED, LOW);
                digitalWrite(YELLOW_LED, LOW);
                digitalWrite(RED_LED, HIGH);
                digitalWrite(BUZZER_PIN, HIGH);

                // Non-blocking incremental soft-start step per tick
                if (currentPwmDuty < 255) {
                    currentPwmDuty = (currentPwmDuty + 25 > 255) ? 255 : currentPwmDuty + 25;
                    ledcWrite(PUMP_PWM_CHANNEL, currentPwmDuty);
                    if (FAN_PIN != -1) ledcWrite(FAN_PWM_CHANNEL, currentPwmDuty);
                }

                if (!smsSentForBreach) {
                    char msg[64];
                    snprintf(msg, sizeof(msg), "ALARM! T:%.1fC, H:%.1f%%, VOC:%d",
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

        // 5. Update 16x2 Display
        lcd.setCursor(0, 0);
        if (dhtError) {
            lcd.print("MODE:ERR DHT ERR ");
        } else {
            lcd.printf("%s T:%.0fC H:%.0f%%", isGuardian ? "GARD" : "INSP", latestMetrics.temp, latestMetrics.humidity);
        }
        lcd.setCursor(0, 1);
        if (mq2Error) {
            lcd.print("MQ2 SENSOR ERR ");
        } else {
            lcd.printf("VOC:%-4d ST:%-5s", latestMetrics.voc, isBreached ? "ALERT" : "OK");
        }
    }
}

// --- Setup ---
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

    // Initialize Event Group & Semaphores
    xSystemEvents = xEventGroupCreate();
    xPowerSleepSemaphore = xSemaphoreCreateBinary();

    // Default system boot state: INSPECT mode
    xEventGroupSetBits(xSystemEvents, MODE_INSPECT_BIT);

    attachInterrupt(digitalPinToInterrupt(INSPECT_BUTTON), isrInspectButton, FALLING);
    attachInterrupt(digitalPinToInterrupt(MODE_BUTTON), isrModeButton, FALLING);
    attachInterrupt(digitalPinToInterrupt(POWER_BUTTON), isrPowerButton, FALLING);

    Wire.begin(SDA_PIN, SCL_PIN);
    lcd.init();
    lcd.backlight();
    dht.begin();

    gsmQueue = xQueueCreate(3, sizeof(char[64]));

    xTaskCreatePinnedToCore(vTaskPowerManagement, "PowerTask", 2048, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(vTaskSystemEngine, "SystemEngine", 4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(vTaskGSM, "GSM_Task", 3072, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(vTaskMQTT, "MQTT_Task", 4096, NULL, 1, NULL, 1);
}

void loop() {
    vTaskDelete(NULL);
}

