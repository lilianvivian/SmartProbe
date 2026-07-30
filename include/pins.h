#ifndef PINS_H
#define PINS_H

// OLED
#define SDA_PIN 21
#define SCL_PIN 20

// GSM
#define GSM_TX_PIN 17
#define GSM_RX_PIN 16

// LEDs
#define GREEN_LED 10
#define YELLOW_LED 11
#define RED_LED 12

// Sensors
#define DHT_PIN 4
#define MQ2_PIN 6

// Only if you decide to switch sensor power
// using a transistor in future
#define SENSOR_ENABLE_PIN   -1

// Fan
// Not wired yet. -1 keeps it off DHT_PIN; fan.cpp no-ops until a pin is set.
#define FAN_PIN -1

// Pump
// Not wired yet - no code drives this.
#define PUMP_PIN 5

// Buzzer
#define BUZZER_PIN 18

// Buttons
#define INSPECT_BUTTON 38
#define MODE_BUTTON 39
#define POWER_BUTTON 40

// Battery
// Unused so far. Shares GPIO 11 with YELLOW_LED - reassign before wiring
// battery sensing.
#define BATTERY_ADC 11

#endif