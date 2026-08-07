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

// --- PWM Configuration ---
#define PUMP_PWM_CHANNEL 0
#define FAN_PWM_CHANNEL   1
#define PWM_FREQ         1000  // 1 kHz
#define PWM_RES          8     // 8-bit resolution (0-255)

// --- Event Group Bit Masks ---
#define MODE_INSPECT_BIT     (1UL << 0UL)  // Default Boot State
#define MODE_WARNING_BIT     (1UL << 1UL)  // Yellow LED + Fan
#define MODE_GUARDIAN_BIT    (1UL << 2UL)  // Sensor Monitoring Active
#define THRESHOLD_BREACH_BIT (1UL << 3UL)  // Red LED + Fan + Buzzer + Pump

#define ALL_MODES_MASK       (MODE_INSPECT_BIT | MODE_WARNING_BIT | MODE_GUARDIAN_BIT)

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

// RTOS Sync Handles & Task Handles
EventGroupHandle_t xSystemEvents = NULL;
QueueHandle_t gsmQueue = NULL;
SemaphoreHandle_t xPowerSleepSemaphore = NULL;
TaskHandle_t xLcdTaskHandle = NULL;  // Dedicated handle for LCD Task

SensorData latestMetrics = {0.0f, 0.0f, 0};
bool smsSentForBreach = false;
uint8_t currentPwmDuty = 0; 

bool globalDhtError = false;
bool globalMq2Error = false;

// --- Helper to Notify LCD Task to Refresh ---
void triggerLCDUpdate() {
    if (xLcdTaskHandle != NULL) {
        xTaskNotifyGive(xLcdTaskHandle);
    }
}

// --- Safe Bidirectional Mode Switching ---
void setSystemMode(EventBits_t modeBit) {
    xEventGroupClearBits(xSystemEvents, ALL_MODES_MASK);
    xEventGroupSetBits(xSystemEvents, modeBit);
    triggerLCDUpdate();
}

// --- Interrupt Service Routines ---
volatile uint32_t lastButtonPress = 0;

void IRAM_ATTR isrInspectButton() { 
    uint32_t now = millis();
    if (now - lastButtonPress > 200) {
        lastButtonPress = now;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xEventGroupClearBitsFromISR(xSystemEvents, ALL_MODES_MASK);
        xEventGroupSetBitsFromISR(xSystemEvents, MODE_INSPECT_BIT, &xHigherPriorityTaskWoken);
        if (xLcdTaskHandle != NULL) {
            vTaskNotifyGiveFromISR(xLcdTaskHandle, &xHigherPriorityTaskWoken);
        }
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void IRAM_ATTR isrModeButton() { 
    uint32_t now = millis();
    if (now - lastButtonPress > 200) {
        lastButtonPress = now;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xEventGroupClearBitsFromISR(xSystemEvents, ALL_MODES_MASK);
        xEventGroupSetBitsFromISR(xSystemEvents, MODE_GUARDIAN_BIT, &xHigherPriorityTaskWoken);
        if (xLcdTaskHandle != NULL) {
            vTaskNotifyGiveFromISR(xLcdTaskHandle, &xHigherPriorityTaskWoken);
        }
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

// --- Dedicated LCD Task (Idle until notification received) ---
void vTaskLCD(void *pvParameters) {
    for (;;) {
        // Blocks completely until xTaskNotifyGive is triggered
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        EventBits_t bits = xEventGroupGetBits(xSystemEvents);
        const char* modeLabel = "INSP";
        
        if (bits & MODE_WARNING_BIT) {
            modeLabel = "WARN";
        } else if (bits & MODE_GUARDIAN_BIT) {
            modeLabel = "GARD";
        }

        bool isBreached = (bits & THRESHOLD_BREACH_BIT) != 0;

        // Line 1 Update
        lcd.setCursor(0, 0);
        if (globalDhtError) {
            lcd.print("MODE:ERR DHT ERR ");
        } else {
            lcd.printf("%-4s T:%2.0fC H:%2.0f%%", modeLabel, latestMetrics.temp, latestMetrics.humidity);
        }

        // Line 2 Update
        lcd.setCursor(0, 1);
        if (globalMq2Error) {
            lcd.print("MQ2 SENSOR ERR  ");
        } else {
            lcd.printf("VOC:%-4d ST:%-5s", latestMetrics.voc, isBreached ? "ALERT" : "OK   ");
        }
    }
}

// --- Hardware Control Helpers ---
void stopActuators() {
    currentPwmDuty = 0;
    ledcWrite(PUMP_PWM_CHANNEL, 0);
    if (FAN_PIN != -1) {
        ledcWrite(FAN_PWM_CHANNEL, 0);
    }
}

void runFanFull() {
    if (FAN_PIN != -1) {
        ledcWrite(FAN_PWM_CHANNEL, 255);
    }
}

// --- Power Management ---
void vTaskPowerManagement(void *pvParameters) {
    for (;;) {
        if (xSemaphoreTake(xPowerSleepSemaphore, portMAX_DELAY) == pdTRUE) {
            vTaskDelay(pdMS_TO_TICKS(50));
            if (digitalRead(POWER_BUTTON) == HIGH) continue;

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

// --- MQTT Engine ---
void mqttCallback(char* topic, byte* payload, unsigned int length) {
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, payload, length);
    if (!err) {
        if (doc.containsKey("mode")) {
            const char* modeStr = doc["mode"];
            if (strcmp(modeStr, "INSPECT") == 0)  setSystemMode(MODE_INSPECT_BIT);
            if (strcmp(modeStr, "WARNING") == 0)  setSystemMode(MODE_WARNING_BIT);
            if (strcmp(modeStr, "GUARDIAN") == 0) setSystemMode(MODE_GUARDIAN_BIT);
        }
        if (doc.containsKey("power") && strcmp(doc["power"], "OFF") == 0) {
            if (xPowerSleepSemaphore != NULL) {
                xSemaphoreGive(xPowerSleepSemaphore);
            }
        }
    }
}

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
                bool isWarning  = (currentBits & MODE_WARNING_BIT) != 0;
                bool isBreached = (currentBits & THRESHOLD_BREACH_BIT) != 0;

                JsonDocument doc;
                doc["mode"]        = isGuardian ? "GUARDIAN" : (isWarning ? "WARNING" : "INSPECT");
                doc["temperature"] = isnan(latestMetrics.temp) ? 0.0 : latestMetrics.temp;
                doc["humidity"]    = isnan(latestMetrics.humidity) ? 0.0 : latestMetrics.humidity;
                doc["voc"]         = latestMetrics.voc;
                doc["breach"]      = isBreached;

                bool isAlert = (isGuardian && isBreached);
                doc["actuators"]["buzzer"] = isAlert;
                doc["actuators"]["pump"]   = isAlert;
                doc["actuators"]["fan"]    = (isWarning || isAlert);

                char buffer[300];
                serializeJson(doc, buffer);
                mqttClient.publish("guardian/telemetry", buffer);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(90000));
    }
}

// --- Non-Blocking GSM Parser ---
void vTaskGSM(void *pvParameters) {
    gsmSerial.begin(9600, SERIAL_8N1, GSM_RX_PIN, GSM_TX_PIN);
    gsmSerial.setTimeout(100);
    vTaskDelay(pdMS_TO_TICKS(1000));

    gsmSerial.println("AT");
    vTaskDelay(pdMS_TO_TICKS(300));
    gsmSerial.println("AT+CMGF=1");
    vTaskDelay(pdMS_TO_TICKS(300));
    gsmSerial.println("AT+CNMI=2,2,0,0,0");
    vTaskDelay(pdMS_TO_TICKS(300));

    char alertMessage[160];

    for (;;) {
        if (xQueueReceive(gsmQueue, &alertMessage, pdMS_TO_TICKS(50)) == pdTRUE) {
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

            if (command.equals("1") || command.indexOf("INSPECT") >= 0) {
                setSystemMode(MODE_INSPECT_BIT);
            } 
            else if (command.equals("2") || command.indexOf("WARNING") >= 0) {
                setSystemMode(MODE_WARNING_BIT);
            } 
            else if (command.equals("3") || command.indexOf("GUARDIAN") >= 0) {
                setSystemMode(MODE_GUARDIAN_BIT);
            }
            else if (command.equals("4") || command.indexOf("POWER=OFF") >= 0) {
                if (xPowerSleepSemaphore != NULL) {
                    xSemaphoreGive(xPowerSleepSemaphore);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// --- Central Control Engine ---
void vTaskSystemEngine(void *pvParameters) {
    TickType_t lastSensorRead = 0;
    const TickType_t SENSOR_INTERVAL = pdMS_TO_TICKS(90000); // 1.5 Minutes

    bool forceRead = true;
    EventBits_t previousBits = 0;

    for (;;) {
        TickType_t now = xTaskGetTickCount();

        // 1. Periodic Sensor Acquisition
        if (forceRead || (now - lastSensorRead >= SENSOR_INTERVAL)) {
            lastSensorRead = now;
            forceRead = false;

            float rawTemp = dht.readTemperature();
            float rawHum  = dht.readHumidity();
            globalDhtError = isnan(rawTemp) || isnan(rawHum);

            latestMetrics.temp = globalDhtError ? 0.0f : rawTemp;
            latestMetrics.humidity = globalDhtError ? 0.0f : rawHum;

            int rawVoc = analogRead(MQ2_PIN);
            globalMq2Error = (rawVoc < 0 || rawVoc > 4095);
            latestMetrics.voc = globalMq2Error ? 0 : rawVoc;

            bool breachDetected = (!globalDhtError && (latestMetrics.temp > TEMP_THRESHOLD || latestMetrics.humidity > HUM_THRESHOLD)) ||
                                  (!globalMq2Error && (latestMetrics.voc > VOC_THRESHOLD));

            if (breachDetected) {
                xEventGroupSetBits(xSystemEvents, THRESHOLD_BREACH_BIT);
            } else {
                xEventGroupClearBits(xSystemEvents, THRESHOLD_BREACH_BIT);
            }

            // Signal LCD Task to Wake Up and render new telemetry data
            triggerLCDUpdate();
        }

        // 2. Evaluate System State & Drive Hardware
        EventBits_t currentBits = xEventGroupWaitBits(
            xSystemEvents,
            ALL_MODES_MASK | THRESHOLD_BREACH_BIT,
            pdFALSE,
            pdFALSE,
            pdMS_TO_TICKS(200)
        );

        if (currentBits != previousBits) {
            previousBits = currentBits;
            triggerLCDUpdate();
        }

        bool isInspect  = (currentBits & MODE_INSPECT_BIT) != 0;
        bool isWarning  = (currentBits & MODE_WARNING_BIT) != 0;
        bool isGuardian = (currentBits & MODE_GUARDIAN_BIT) != 0;
        bool isBreached = (currentBits & THRESHOLD_BREACH_BIT) != 0;

        // --- MODE 1: INSPECT (Default Standby) ---
        if (isInspect) { 
            digitalWrite(GREEN_LED, HIGH);
            digitalWrite(YELLOW_LED, LOW);
            digitalWrite(RED_LED, LOW);
            digitalWrite(BUZZER_PIN, LOW);
            stopActuators();
            smsSentForBreach = false;
        } 
        // --- MODE 2: WARNING (Yellow LED + Fan) ---
        else if (isWarning) {
            digitalWrite(GREEN_LED, LOW);
            digitalWrite(YELLOW_LED, HIGH);
            digitalWrite(RED_LED, LOW);
            digitalWrite(BUZZER_PIN, LOW);
            
            ledcWrite(PUMP_PWM_CHANNEL, 0);
            runFanFull();
            smsSentForBreach = false;
        }
        // --- MODE 3: GUARDIAN (Monitoring Mode) ---
        else if (isGuardian) { 
            if (isBreached) {
                digitalWrite(GREEN_LED, LOW);
                digitalWrite(YELLOW_LED, LOW);
                digitalWrite(RED_LED, HIGH);
                digitalWrite(BUZZER_PIN, HIGH);

                runFanFull();

                if (currentPwmDuty < 255) {
                    currentPwmDuty = (currentPwmDuty + 25 > 255) ? 255 : currentPwmDuty + 25;
                    ledcWrite(PUMP_PWM_CHANNEL, currentPwmDuty);
                }

                if (!smsSentForBreach) {
                    char msg[160];
                    snprintf(msg, sizeof(msg), 
                        "ALARM BREACH! T:%.1fC H:%.1f%% VOC:%d\r\nReply Option:\r\n1-Inspect\r\n2-Warning\r\n3-Guardian\r\n4-Power OFF",
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

    xSystemEvents = xEventGroupCreate();
    xPowerSleepSemaphore = xSemaphoreCreateBinary();

    // Default System Boot Mode: INSPECT
    xEventGroupSetBits(xSystemEvents, MODE_INSPECT_BIT);

    attachInterrupt(digitalPinToInterrupt(INSPECT_BUTTON), isrInspectButton, FALLING);
    attachInterrupt(digitalPinToInterrupt(MODE_BUTTON), isrModeButton, FALLING);
    attachInterrupt(digitalPinToInterrupt(POWER_BUTTON), isrPowerButton, FALLING);

    Wire.begin(SDA_PIN, SCL_PIN);
    lcd.init();
    lcd.backlight();
    dht.begin();

    gsmQueue = xQueueCreate(3, sizeof(char[160]));

    // --- Task Creation ---
    xTaskCreatePinnedToCore(vTaskLCD, "LCD_Task", 2048, NULL, 1, &xLcdTaskHandle, 1);
    xTaskCreatePinnedToCore(vTaskPowerManagement, "PowerTask", 2048, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(vTaskSystemEngine, "SystemEngine", 4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(vTaskGSM, "GSM_Task", 3072, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(vTaskMQTT, "MQTT_Task", 4096, NULL, 1, NULL, 1);

    // Initial boot render
    triggerLCDUpdate();
}

void loop() {
    vTaskDelete(NULL);
}