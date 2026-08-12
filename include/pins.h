#ifndef PINS_H
#define PINS_H

#include <Arduino.h>

// LCD Display (I2C LiquidCrystal)
#define SDA_PIN             21
#define SCL_PIN             20

// GSM Module (UART2)
#define GSM_TX_PIN          17
#define GSM_RX_PIN          16

// Indicator LEDs
#define GREEN_LED           10
#define YELLOW_LED          11
#define RED_LED             12

// Sensors
#define DHT_PIN             4      // DHT11 Sensor
#define DHT_TYPE            DHT11  
#define MQ2_PIN             6

// Actuators
#define FAN_PIN             42    // Unassigned (-1 safe handling)
#define PUMP_PIN             5
#define BUZZER_PIN          18

// Push Buttons
#define INSPECT_BUTTON      38
#define MODE_BUTTON         39
#define POWER_BUTTON        15    // RTC GPIO used for Deep Sleep & Wakeup

// Battery ADC (Disabled to fix pin conflict with GPIO 11)
#define BATTERY_ADC         -1

// Alarm Thresholds & GSM Target
#define TEMP_THRESHOLD      32.0f  // °C
#define HUM_THRESHOLD       65.0f  // %
#define VOC_THRESHOLD       1200    // Raw ADC
#define TARGET_PHONE_NUM    "+254702808564"

// Wi-Fi & MQTT Configuration
#define WIFI_SSID           "Galaxy A126A3D"
#define WIFI_PASS           "Nasimiyu1200"
#define MQTT_BROKER_IP      "192.168.40.66" // IP of your laptop/server
#define MQTT_PORT           1883

#endif // PINS_H