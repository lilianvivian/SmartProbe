#ifndef SECRETS_H
#define SECRETS_H

//--------------------------------------------------
// Secrets / deployment configuration
//--------------------------------------------------
// This file is gitignored. See secrets.example.h for the template.
// Fill in the WiFi network the probe will join and your MQTT broker.
//--------------------------------------------------

// WiFi
#define WIFI_SSID   "iPhone"
#define WIFI_PASS   "12345678"

// MQTT broker
// Quick demo: a public broker (test.mosquitto.org / broker.hivemq.com).
// Phase 2: point this at your own Mosquitto instance.
#define MQTT_HOST   "broker.hivemq.com"
#define MQTT_PORT   1883

// Optional broker auth (empty = anonymous)
#define MQTT_USER   ""
#define MQTT_PASS   ""

#endif
