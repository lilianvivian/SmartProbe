#ifndef SECRETS_H
#define SECRETS_H

//--------------------------------------------------
// Secrets / deployment configuration
//--------------------------------------------------
// Copy this file to "secrets.h" and fill in your values.
// secrets.h is gitignored so real credentials never get committed.
//--------------------------------------------------

// WiFi — the network the probe will join
#define WIFI_SSID   "YOUR_WIFI_SSID"
#define WIFI_PASS   "YOUR_WIFI_PASSWORD"

// MQTT broker
// Quick demo: use a public broker (test.mosquitto.org / broker.hivemq.com).
// Phase 2: point this at your own Mosquitto instance.
#define MQTT_HOST   "broker.hivemq.com"
#define MQTT_PORT   1883

// Optional broker auth (leave empty for anonymous/public brokers)
#define MQTT_USER   ""
#define MQTT_PASS   ""

#endif
