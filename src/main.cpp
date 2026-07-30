#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_Sensor.h>
#include <VL53L1X.h>
#include <DHT.h>
#include <ESP32Servo.h> // Using the requested ESP32Servo Library

// ============================================================================
// --- FORWARD DECLARATION STRUCTURES (Placed at top to resolve compile scope) ---
// ============================================================================
typedef struct __attribute__((packed)) {
    int id;           // 1 for Node A
    int joyX;         
    int joyY;         
    bool btnState;    
    int intensity;    
} tx_message_t;

typedef struct __attribute__((packed)) {
    float temperature;   
    float humidity;      
    float pressurePa;    
    float baroAltitude;  
    float accelX;        
    float accelY;
    float accelZ;
    float laserDistance; 
    int batteryPct;      

    bool dhtAlive;     
    bool mpuAlive;
    bool baroAlive;
    bool tofAlive;
} rx_message_t;

// ============================================================================
// --- Hardware Pin Layout Configurations ---
// ============================================================================
#define PIN_DHT      17
#define DHT_TYPE     DHT22  
#define I2C_SDA      21
#define I2C_SCL      16
#define PIN_SERVO    32
#define PIN_ADC_35   35


// Functional Hardware Connection State Variables
bool isDhtReady  = false;
bool isMpuReady  = false;
bool isBaroReady = false;
bool isTofReady  = false;

// Targeted Remote Hardware Node Mac Addresses
uint8_t macA[] = {0xec, 0x62, 0x60, 0xa7, 0x1b, 0xe8};
uint8_t macC[] = {0x08, 0xa6, 0xf7, 0x12, 0x7f, 0x38};

volatile bool btnStateMode = false;
volatile int targetServoAngle = 90;

// Instantiate Objects
Adafruit_MPU6050 mpu;
Adafruit_BMP085  bmp; 
VL53L1X          vl53;
DHT              dht(PIN_DHT, DHT_TYPE);
Servo            radarServo; // ESP32Servo Object Instance
QueueHandle_t    relayQueue = NULL;

// ============================================================================
// --- Network Data Intercept Hook ---
// ============================================================================
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingDataBytes, int len) {
    if (len == sizeof(tx_message_t)) {
        tx_message_t receivedJoystick;
        memcpy(&receivedJoystick, incomingDataBytes, sizeof(tx_message_t));

        btnStateMode = receivedJoystick.btnState;
        if(btnStateMode){
            targetServoAngle = map(receivedJoystick.joyX, 0, 4095, 10, 170);
            targetServoAngle = constrain(targetServoAngle, 10, 170);
        }

        xQueueSendFromISR(relayQueue, &receivedJoystick, NULL);
    }
}

// ============================================================================
// --- Task 1: Continuous Input Relay Interface (Node B -> Node C) ---
// ============================================================================
void vTaskRelay(void *pvParameters) {
    tx_message_t joystickData;
    for (;;) {
        if (xQueueReceive(relayQueue, &joystickData, portMAX_DELAY) == pdPASS) {
            esp_now_send(macC, (uint8_t *)&joystickData, sizeof(tx_message_t));
        }
    }
}

// ============================================================================
// --- Task 2: Advanced Telemetry Mapping (Node B -> Node A Return Path) ---
// ============================================================================
void vTaskSensors(void *pvParameters) {
    rx_message_t telemetry;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    const float DIVIDER_RATIO = (47000.0f + 10000.0f) / 10000.0f; 
    const float ESP32_REF_VOLTS = 3.3f;                           
    const float BATT_MAX_VOLTS = 16.0f; const float BATT_MIN_VOLTS = 14.0f;
    
    const TickType_t xFrequency = pdMS_TO_TICKS(500); 

    for (;;) {
        telemetry.dhtAlive  = isDhtReady;
        telemetry.mpuAlive  = isMpuReady;
        telemetry.baroAlive = isBaroReady;
        telemetry.tofAlive  = isTofReady;

        if (isDhtReady) {
            float h = dht.readHumidity();
            float t = dht.readTemperature();
            telemetry.humidity = isnan(h) ? 0.0f : h;
            telemetry.temperature = isnan(t) ? 0.0f : t;
        } else {
            telemetry.humidity = 0.0f; telemetry.temperature = 0.0f;
        }

        if (isMpuReady) {
            sensors_event_t a, g, temp;
            mpu.getEvent(&a, &g, &temp);
            telemetry.accelX = a.acceleration.x;
            telemetry.accelY = a.acceleration.y;
            telemetry.accelZ = a.acceleration.z;
        } else {
            telemetry.accelX = 0.0f; telemetry.accelY = 0.0f; telemetry.accelZ = 0.0f;
        }

        if (isBaroReady) {
            telemetry.pressurePa = (float)bmp.readPressure();
            telemetry.baroAltitude = bmp.readAltitude(101325); 
            if (!isDhtReady) { 
                telemetry.temperature = bmp.readTemperature(); 
            }
        } else {
            telemetry.pressurePa = 0.0f; telemetry.baroAltitude = 0.0f;
        }

        if (isTofReady) {
            uint16_t distMm = vl53.read();
            telemetry.laserDistance = (distMm == 65535) ? -1.0f : (float)distMm / 10.0f;
        } else {
            telemetry.laserDistance = -1.0f;
        }

        
        int rawADC35 = analogRead(PIN_ADC_35);
        float pinVoltage = (rawADC35 / 4095.0f) * ESP32_REF_VOLTS;
        float batteryVoltage = pinVoltage * DIVIDER_RATIO;
        float pctFloat = ((batteryVoltage - BATT_MIN_VOLTS) / (BATT_MAX_VOLTS - BATT_MIN_VOLTS)) * 100.0f;
        telemetry.batteryPct = constrain((int)pctFloat, 0, 100);

        esp_now_send(macA, (uint8_t *)&telemetry, sizeof(rx_message_t));

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ============================================================================
// --- Task 3: Non-Blocking Multi-Directional Radar Servo Sweep ---
// ============================================================================
void vTaskServo(void *pvParameters) {
  
    radarServo.write(90);
    vTaskDelay(pdMS_TO_TICKS(700));

    for(;;){
        if(btnStateMode){
            radarServo.write(targetServoAngle);
        }
    }

        // Suspend task non-blockingly to yield cycle allocations
        vTaskDelay(pdMS_TO_TICKS(20)); 
    }


// ============================================================================
// --- Core System Setup ---
// ============================================================================
void setup() {
    Serial.begin(115200);

    // Initialise Shared I2C Control Infrastructure 
    Wire.begin(I2C_SDA, I2C_SCL, 100000);

    Serial.println("\n=== Initializing GY-87 & Shared Hardware Array ===");

    // [Safety Wrapper 1]: DHT Check
    dht.begin();
    if (!isnan(dht.readTemperature())) {
        isDhtReady = true;
        Serial.println("[OK] DHT22 Operational.");
    } else {
        Serial.println("[FAIL] DHT22 Connection Timed Out.");
    }

    // [Safety Wrapper 2]: GY-87 MPU6050 Check
    if (mpu.begin(0x68, &Wire)) {
        isMpuReady = true;
        Serial.println("[OK] GY-87 IMU (MPU6050) Connected.");
    } else {
        Serial.println("[FAIL] GY-87 IMU (MPU6050) Missing.");
    }

    // [Safety Wrapper 3]: GY-87 BMP180 Check
    if (bmp.begin()) {
        isBaroReady = true;
        Serial.println("[OK] GY-87 Barometer (BMP180) Connected.");
    } else {
        Serial.println("[FAIL] GY-87 Barometer (BMP180) Missing.");
    }

    // [Safety Wrapper 4]: VL53L1X Check
    vl53.setTimeout(200);
    if (vl53.init()) {
        isTofReady = true;
        vl53.setDistanceMode(VL53L1X::Long);
        vl53.setMeasurementTimingBudget(40000);
        vl53.startContinuous(50);
        Serial.println("[OK] VL53L1X TOF Distance Module Active.");
    } else {
        Serial.println("[FAIL] VL53L1X TOF Module Offline.");
    }
    Serial.println("=============================================\n");

    // Initialize ESP32Servo properties safely outside loop structure
    radarServo.setPeriodHertz(50);
    radarServo.attach(PIN_SERVO, 500, 2500);

    // Boot Up Networking Hardware Layer
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) return;

    esp_now_register_recv_cb(OnDataRecv);

    esp_now_peer_info_t peerInfo = {};
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    
    memcpy(peerInfo.peer_addr, macA, 6); esp_now_add_peer(&peerInfo);
    memcpy(peerInfo.peer_addr, macC, 6); esp_now_add_peer(&peerInfo);

    relayQueue = xQueueCreate(10, sizeof(tx_message_t));

    // Spawn Multithread Processing Allocations across Core 1
    xTaskCreatePinnedToCore(vTaskRelay, "RelayEngine", 4096, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(vTaskSensors, "SensorEngine", 4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(vTaskServo, "ServoDriver", 2048, NULL, 1, NULL, 1);
}

void loop() {
    vTaskDelete(NULL); 
}


