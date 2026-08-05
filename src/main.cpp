
#include <Arduino.h>
#include <DHT.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <freertos/semphr.h>
#include "pins.h"
#include <driver/rtc_io.h>
// --- PWM Configuration for Inductive Load Soft-Start ---
#define PUMP_PWM_CHANNEL 0
#define FAN_PWM_CHANNEL  1
#define PWM_FREQ         1000  // 1 kHz
#define PWM_RES          8     // 8-bit resolution (0-255)
// --- Enums & Data Structures ---
enum SystemState {
    MODE_INSPECT,  // Passive monitoring: Metrics displayed, Actuators strictly OFF
    MODE_GUARDIAN  // Active protection: Actuators & Alarms trigger on threshold breach
};
struct SensorData {
    float temp;
    float humidity;
    int voc;
    bool thresholdBreached;
};
// --- Globals & Handles ---
volatile SystemState currentState = MODE_INSPECT;
DHT dht(DHT_PIN, DHT_TYPE);
LiquidCrystal_I2C lcd(0x27, 16, 2); 
HardwareSerial gsmSerial(2); // Hardware UART2
WiFiClient espClient;PubSubClient mqttClient(espClient);
QueueHandle_t gsmQueue;SemaphoreHandle_t xPowerSleepSemaphore = NULL; // Semaphore for Deferred Sleep Handling
SensorData latestMetrics = {0.0f, 0.0f, 0, false};
// Flags to regulate timing and prevent continuous loops
bool smsSentForBreach = false;
bool actuatorsCurrentlyActive = false;
// --- Interrupt Service Routines (ISRs) ---
void IRAM_ATTR isrInspectButton() { 
    currentState = MODE_INSPECT; 
}
void IRAM_ATTR isrModeButton() { 
    currentState = MODE_GUARDIAN; 
}
// Lightweight Power Button ISR - Defers actual sleep processing to an RTOS taskvoid 
void IRAM_ATTR isrPowerButton() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (xPowerSleepSemaphore != NULL) {
        xSemaphoreGiveFromISR(xPowerSleepSemaphore, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken) {
            portYIELD_FROM_ISR();
        }
    }
}
// --- Actuator Safe-Control Helper Functions ---
void stopActuators() {
    ledcWrite(PUMP_PWM_CHANNEL, 0);
    if (FAN_PIN != -1) {
        ledcWrite(FAN_PWM_CHANNEL, 0);
    }
    if (actuatorsCurrentlyActive) {
        actuatorsCurrentlyActive = false;
        Serial.println("[ACTUATOR] Motors deactivated cleanly.");
    }
}
void startActuatorsSoftly() {
    // If already running at max speed, skip to keep task loop non-blocking
    if (actuatorsCurrentlyActive) return; 
    
    Serial.println("[ACTUATOR] Running smooth soft-start to eliminate current spike...");
    // Gradually ramp the duty cycle from 0 to 255 over ~750ms
    for (int duty = 0; duty <= 255; duty += 5) {
        ledcWrite(PUMP_PWM_CHANNEL, duty);
        if (FAN_PIN != -1) {
            ledcWrite(FAN_PWM_CHANNEL, duty);
        }
        vTaskDelay(pdMS_TO_TICKS(15)); // Yields CPU control to prevent blocking other network tasks
    }
    actuatorsCurrentlyActive = true;
    Serial.println("[ACTUATOR] Soft-start complete. Motors locked at full safe velocity.");
}
// --- Deferred Power Management Task (Core 0) ---
void vTaskPowerManagement(void *pvParameters) {
    for (;;) {
        if (xSemaphoreTake(xPowerSleepSemaphore, portMAX_DELAY) == pdTRUE) {

            vTaskDelay(pdMS_TO_TICKS(50));

            if(digitalRead(POWER_BUTTON) == HIGH){
                Serial.println("[POWER] Suppressed false sleep trigger(Electrical Noise/Glitch).");
                continue;
            }

            Serial.println("[POWER] Valid press confirmed.Preparing Deep Sleep ...");

            // 1. Wait until user RELEASES the button so we don't wake up immediately
            while (digitalRead(POWER_BUTTON) == LOW) {
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            vTaskDelay(pdMS_TO_TICKS(200)); // Debounce buffer

            // 2. Shut down peripherals
            digitalWrite(GREEN_LED, LOW);
            digitalWrite(YELLOW_LED, LOW);
            digitalWrite(RED_LED, LOW);
            digitalWrite(BUZZER_PIN, LOW);
            stopActuators();

            lcd.noBacklight();
            lcd.clear();

            // 3. Keep RTC IO Domain powered during deep sleep (REQUIRED for ESP32-S3 pull-ups)
            esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);

            // 4. Configure RTC Pin state and force internal pull-up to stay ACTIVE during sleep
            rtc_gpio_init((gpio_num_t)POWER_BUTTON);
            rtc_gpio_set_direction((gpio_num_t)POWER_BUTTON, RTC_GPIO_MODE_INPUT_ONLY);
            rtc_gpio_pullup_en((gpio_num_t)POWER_BUTTON);
            rtc_gpio_pulldown_dis((gpio_num_t)POWER_BUTTON);

            // 5. Tell RTC to hold this pin state so it doesn't drop to 0V during sleep
            gpio_hold_en((gpio_num_t)POWER_BUTTON);

            // 6. Enable ext0 Wakeup on LOW state
            esp_sleep_enable_ext0_wakeup((gpio_num_t)POWER_BUTTON, 0); // 0 = Wake on LOW

            Serial.println("[POWER] Going to sleep now. Press button to wake!");
            Serial.flush();
            esp_deep_sleep_start();
        }
    }
}
// --- MQTT Callback (Handling Dashboard Remote Control) ---
void mqttCallback(char* topic, byte* payload, unsigned int length) {
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, payload, length);
    if (!err) {
        if (doc.containsKey("mode")) {
            const char* modeStr = doc["mode"];
            if (strcmp(modeStr, "INSPECT") == 0)   currentState = MODE_INSPECT;
            if (strcmp(modeStr, "GUARDIAN") == 0)  currentState = MODE_GUARDIAN;
        }
        if (doc.containsKey("power") && strcmp(doc["power"], "OFF") == 0) {
            if (xPowerSleepSemaphore != NULL) {
                xSemaphoreGive(xPowerSleepSemaphore);
            }
        }
    }
}
// --- Task 3: MQTT Dashboard Sync (Core 1) ---
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

                // Construct Telemetry JSON Payload
                JsonDocument doc;
                doc["mode"]        = (currentState == MODE_INSPECT) ? "INSPECT" : "GUARDIAN";
                doc["temperature"] = isnan(latestMetrics.temp) ? 0.0 : latestMetrics.temp;
                doc["humidity"]    = isnan(latestMetrics.humidity) ? 0.0 : latestMetrics.humidity;
                doc["voc"]         = latestMetrics.voc;
                doc["breach"]      = latestMetrics.thresholdBreached;

                // Active state of actuators
                bool actuatorActive = (currentState == MODE_GUARDIAN && latestMetrics.thresholdBreached);
                doc["actuators"]["buzzer"] = actuatorActive;
                doc["actuators"]["pump"]   = actuatorActive;
                doc["actuators"]["fan"]    = (FAN_PIN != -1) ? actuatorActive : false;

                char buffer[300];
                serializeJson(doc, buffer);
                mqttClient.publish("guardian/telemetry", buffer);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // Publish payload every 1 second
    }
}
// --- Task 2: GSM Engine (Core 1) ---
void vTaskGSM(void *pvParameters) {
    gsmSerial.begin(9600, SERIAL_8N1, GSM_RX_PIN, GSM_TX_PIN);
    vTaskDelay(pdMS_TO_TICKS(1000));

    gsmSerial.println("AT");
    vTaskDelay(pdMS_TO_TICKS(500));
    gsmSerial.println("AT+CMGF=1");                  // Set SMS to Text Mode
    vTaskDelay(pdMS_TO_TICKS(500));
    gsmSerial.println("AT+CNMI=2,2,0,0,0");          // Route incoming SMS directly to Serial
    vTaskDelay(pdMS_TO_TICKS(500));

    char alertMessage[64];

    for (;;) {
        // 1. Handle outgoing queued SMS requests
        if (xQueueReceive(gsmQueue, &alertMessage, pdMS_TO_TICKS(100)) == pdTRUE) {
            gsmSerial.println("AT+CMGF=1");
            vTaskDelay(pdMS_TO_TICKS(200));
            gsmSerial.printf("AT+CMGS=\"%s\"\r\n", TARGET_PHONE_NUM);
            vTaskDelay(pdMS_TO_TICKS(200));
            gsmSerial.print(alertMessage);
            gsmSerial.write(26); // CTRL+Z to send
            vTaskDelay(pdMS_TO_TICKS(3000));
        }

        // 2. Parse incoming remote SMS commands
        if (gsmSerial.available()) {
            String incoming = gsmSerial.readString();
            Serial.print("[GSM INCOMING]: ");
            Serial.println(incoming);

            if (incoming.indexOf("MODE=INSPECT") >= 0)  currentState = MODE_INSPECT;
            if (incoming.indexOf("MODE=GUARDIAN") >= 0) currentState = MODE_GUARDIAN;
            if (incoming.indexOf("POWER=OFF") >= 0) {
                if (xPowerSleepSemaphore != NULL) {
                    xSemaphoreGive(xPowerSleepSemaphore);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
// --- Task 1: System Engine & Safety Interlocks (Core 0) ---
void vTaskSystemEngine(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        // --- 1. Read & Validate DHT11 Sensor ---
        float rawTemp = dht.readTemperature();
        float rawHum  = dht.readHumidity();
        bool dhtError = false;

        if (isnan(rawTemp) || isnan(rawHum)) {
            dhtError = true;
            Serial.println("[ERROR] DHT11 Read Failed! Sensor disconnected or timing timeout.");
            latestMetrics.temp = 0.0f;
            latestMetrics.humidity = 0.0f;
        } else {
            latestMetrics.temp = rawTemp;
            latestMetrics.humidity = rawHum;
            Serial.printf("[SENSOR OK] Temp: %.1f C | Hum: %.1f %%\n", latestMetrics.temp, latestMetrics.humidity);
        }

        // --- 2. Read & Validate MQ2 Sensor ---
        int rawVoc = analogRead(MQ2_PIN);
        bool mq2Error = false;

        if (rawVoc < 0 || rawVoc > 4095) {
            mq2Error = true;
            Serial.println("[ERROR] MQ2 Sensor Read Out of Range!");
            latestMetrics.voc = 0;
        } else {
            latestMetrics.voc = rawVoc;

Serial.printf("[SENSOR OK] MQ2 VOC Raw ADC: %d\n", latestMetrics.voc);
}
// --- 3. Evaluate Thresholds ---
latestMetrics.thresholdBreached = (!dhtError && (latestMetrics.temp > TEMP_THRESHOLD || latestMetrics.humidity > HUM_THRESHOLD)) ||
(!mq2Error && (latestMetrics.voc > VOC_THRESHOLD));
// --- 4. State Machine Logic & Actuator Interlocks ---
if (currentState == MODE_INSPECT) {
digitalWrite(GREEN_LED, HIGH);
digitalWrite(YELLOW_LED, LOW);
digitalWrite(RED_LED, LOW);
digitalWrite(BUZZER_PIN, LOW);
stopActuators();
smsSentForBreach = false;
}
else if (currentState == MODE_GUARDIAN) {
if (latestMetrics.thresholdBreached) {
digitalWrite(GREEN_LED, LOW);
digitalWrite(YELLOW_LED, LOW);
digitalWrite(RED_LED, HIGH);
digitalWrite(BUZZER_PIN, HIGH);
// Safe Soft-Start Trigger
startActuatorsSoftly();
// Send Outbound SMS Alert ONCE per breach event
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
// --- 5. Update 16x2 I2C LCD Display ---
lcd.setCursor(0, 0);
if (dhtError) {
lcd.print("MODE:ERR DHT ERR ");
} else {
lcd.printf("%s T:%.0fC H:%.0f%%",
(currentState == MODE_INSPECT) ? "INSP" : "GARD",
latestMetrics.temp,
latestMetrics.humidity);
}
lcd.setCursor(0, 1);
if (mq2Error) {
lcd.print("MQ2 SENSOR ERR ");
} else {
lcd.printf("VOC:%-4d ST:%-5s",
latestMetrics.voc,
latestMetrics.thresholdBreached ? "ALERT" : "OK");
}
vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(2000));
}
}
// --- Setup ---
void setup() {
Serial.begin(115200);
// Initialize Standard Indicators
pinMode(GREEN_LED, OUTPUT);
pinMode(YELLOW_LED, OUTPUT);
pinMode(RED_LED, OUTPUT);
pinMode(BUZZER_PIN, OUTPUT);
// Bind Motor Driver Pins to Modern ESP32 Hardware LEDC (PWM) Peripherals
ledcSetup(PUMP_PWM_CHANNEL, PWM_FREQ, PWM_RES);
ledcAttachPin(PUMP_PIN, PUMP_PWM_CHANNEL);
if (FAN_PIN != -1) {
ledcSetup(FAN_PWM_CHANNEL, PWM_FREQ, PWM_RES);
ledcAttachPin(FAN_PIN, FAN_PWM_CHANNEL);
}
// Initialize Push Buttons
pinMode(INSPECT_BUTTON, INPUT_PULLUP);
pinMode(MODE_BUTTON, INPUT_PULLUP);
pinMode(POWER_BUTTON, INPUT_PULLUP);
// Create Deferred Binary Semaphore for Power Handling
xPowerSleepSemaphore = xSemaphoreCreateBinary();
// Hardware Button Interrupts
attachInterrupt(digitalPinToInterrupt(INSPECT_BUTTON), isrInspectButton, FALLING);
attachInterrupt(digitalPinToInterrupt(MODE_BUTTON), isrModeButton, FALLING);
attachInterrupt(digitalPinToInterrupt(POWER_BUTTON), isrPowerButton, FALLING);
// Initialize LCD & DHT11
Wire.begin(SDA_PIN, SCL_PIN);
lcd.init();
lcd.backlight();
dht.begin();
// Create Inter-Task Communication Queue
gsmQueue = xQueueCreate(3, sizeof(char[64]));
// Spawn Core FreeRTOS Tasks
xTaskCreatePinnedToCore(vTaskPowerManagement, "PowerTask", 2048, NULL, 3, NULL, 0);
xTaskCreatePinnedToCore(vTaskSystemEngine, "SystemEngine", 4096, NULL, 2, NULL, 0);
xTaskCreatePinnedToCore(vTaskGSM, "GSM_Task", 3072, NULL, 1, NULL, 1);
xTaskCreatePinnedToCore(vTaskMQTT, "MQTT_Task", 4096, NULL, 1, NULL, 1);
}
void loop() {
// Delete setup/loop task to maximize stack resource allocation
vTaskDelete(NULL);
}
