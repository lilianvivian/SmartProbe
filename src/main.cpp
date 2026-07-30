#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <math.h>

// --- Pin Configurations ---
#define PIN_VRX 34  // X-Axis Analog Input
#define PIN_VRY 35  // Y-Axis Analog Input
#define PIN_SW  27  // Pushbutton Switch (Requires Pull-up)
#define PIN_LED 2   // Onboard / External Feedback LED (PWM output)

// --- PWM Configurations ---
#define PWM_FREQ     5000
#define PWM_CHAN     0
#define PWM_RES      8  // 8-bit resolution (0-255)

// --- Joystick Calibration Defaults ---
#define JOY_CENTER 1875  
#define JOY_DEADZONE 150 

// Target Node B (Hub) MAC Address
uint8_t macB[] = {0x68, 0x09, 0x47, 0x5d, 0x14, 0xbc};

// --- Global Buffer for Real-time Local Control Values ---
// Declared volatile to ensure thread-safe cross-task memory reads
volatile int currentLocalJoyX = JOY_CENTER;
volatile int currentLocalJoyY = JOY_CENTER;

// --- Strict Network Packet Definitions (Packed for 0-Byte Structure Padding) ---
typedef struct __attribute__((packed)) {
    int id;           // 1 for Node A
    int joyX;         
    int joyY;         
    bool btnState;    
    int intensity;    
} tx_message_t;

typedef struct __attribute__((packed)) {
    // Atmospheric & Multi-Sensor Telemetry Block
    float temperature;   // Read from DHT22 or BMP180
    float humidity;      // Read from DHT22
    float pressurePa;    // Barometric Pressure (Pascals) from GY-87 (BMP180)
    float baroAltitude;  // Estimated Altitude (Meters) calculated from pressure
    float accelX;        // Motion readings from GY-87 (MPU6050)
    float accelY;
    float accelZ;
    float laserDistance; // Ranging distance (cm) from independent VL53L1X module
    int batteryPct;      // Remote battery metrics mapped from 14V-16V window

    // Component-Specific Hardware Connection Flags
    bool dhtAlive;     
    bool mpuAlive;
    bool baroAlive;
    bool tofAlive;
} rx_message_t;

// --- FreeRTOS Queue Handles ---
QueueHandle_t rxQueue = NULL;

// --- ESP-NOW Callbacks ---
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingDataBytes, int len) {
    rx_message_t incomingPacket;
    if (len == sizeof(rx_message_t)) {
        memcpy(&incomingPacket, incomingDataBytes, sizeof(rx_message_t));
        xQueueSendFromISR(rxQueue, &incomingPacket, NULL);
    }
}

// --- Task 1: Non-Delay Continuous Controller Sampling & Transmission ---
void vTaskSender(void *pvParameters) {
    tx_message_t txData;
    txData.id = 1;
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(50); // Continuous 50ms stream

    for (;;) {
        int rawX = analogRead(PIN_VRX);
        int rawY = analogRead(PIN_VRY);
        bool rawSW = !digitalRead(PIN_SW);

        // Invert axes to correct physical steering and driving mapping profiles
        int invertedY = 4095 - rawY; 
        int invertedX = 4095 - rawX; 

        // Update tracking channels for the logger console thread safely
        currentLocalJoyX = invertedX;
        currentLocalJoyY = invertedY;

        long deltaX = invertedX - JOY_CENTER;
        long deltaY = invertedY - JOY_CENTER;
        double distanceVec = sqrt((deltaX * deltaX) + (deltaY * deltaY));

        int ledBrightness = 0;
        if (distanceVec > JOY_DEADZONE) {
            ledBrightness = map(distanceVec, JOY_DEADZONE, 2000, 0, 255);
            ledBrightness = constrain(ledBrightness, 0, 255);
        }

        ledcWrite(PWM_CHAN, ledBrightness);

        txData.joyX = invertedX;
        txData.joyY = invertedY; 
        txData.btnState = rawSW;
        txData.intensity = ledBrightness;

        esp_now_send(macB, (uint8_t *)&txData, sizeof(tx_message_t));

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// --- Task 2: Advanced Telemetry Queue Processing & Integrated CSV Output ---
void vTaskReceiver(void *pvParameters) {
    rx_message_t rxData;

    // UPDATED CSV HEADER: Added local_joy_x and local_joy_y right after the timestamp
    Serial.println("\ntimestamp_ms,local_joy_x,local_joy_y,dht_ok,mpu_ok,bmp_ok,tof_ok,temp_c,humidity_pct,pressure_pa,altitude_m,accel_x,accel_y,accel_z,laser_dist_cm,battery_pct");

    for (;;) {
        if (xQueueReceive(rxQueue, &rxData, portMAX_DELAY) == pdPASS) {
            
            // Print clean row combining live local controller sticks and incoming telemetry
            Serial.printf("%lu,%d,%d,%d,%d,%d,%d,%.2f,%.2f,%.1f,%.2f,%.3f,%.3f,%.3f,%.2f,%d\n",
                          millis(),
                          currentLocalJoyX, // Integrated Analog Parameters
                          currentLocalJoyY,
                          rxData.dhtAlive  ? 1 : 0,
                          rxData.mpuAlive  ? 1 : 0,
                          rxData.baroAlive ? 1 : 0,
                          rxData.tofAlive  ? 1 : 0,
                          rxData.temperature,
                          rxData.humidity,
                          rxData.pressurePa,
                          rxData.baroAltitude,
                          rxData.accelX,
                          rxData.accelY,
                          rxData.accelZ,
                          rxData.laserDistance,
                          rxData.batteryPct);
        }
    }
}

void setup() {
    Serial.begin(115200);

    pinMode(PIN_SW, INPUT_PULLUP);
    ledcSetup(PWM_CHAN, PWM_FREQ, PWM_RES);
    ledcAttachPin(PIN_LED, PWM_CHAN);

    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {
        Serial.println("Fatal Error: ESP-NOW Architecture Failed to Init.");
        return;
    }

    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, macB, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Fatal Error: Failed to add Node B peer structure map.");
        return;
    }

    rxQueue = xQueueCreate(5, sizeof(rx_message_t));
    if (rxQueue == NULL) return;

    xTaskCreatePinnedToCore(vTaskSender, "JoystickSender", 3072, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(vTaskReceiver, "ConsoleReceiver", 4096, NULL, 1, NULL, 1);
}

void loop() {
    vTaskDelete(NULL); 
}
