#ifndef PINS_H
#define PINS_H

// OLED
#define SDA_PIN 21
#define SCL_PIN 20

// GSM
#define GSM_TX_PIN 9
#define GSM_RX_PIN 10

// LEDs
#define GREEN_LED 15
#define YELLOW_LED 16
#define RED_LED 17

// Only if you decide to switch sensor power
// using a transistor in future
#define SENSOR_ENABLE_PIN   -1

// Fan
#define FAN_PIN 4

// Pump
#define PUMP_PIN 5

// Buzzer
#define BUZZER_PIN 18

// Buttons
#define INSPECT_BUTTON 35
#define MODE_BUTTON 36
#define POWER_BUTTON 37

// Battery
#define BATTERY_ADC 11

#endif