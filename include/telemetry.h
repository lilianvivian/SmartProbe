#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "systemreport.h"

//--------------------------------------------------
// Telemetry
//--------------------------------------------------
// Owns the WiFi + MQTT connection and publishes each inspection
// as JSON so the live dashboard (and, in Phase 2, a time-series DB)
// can subscribe. All calls are non-blocking and safe to make even
// when the network is down — the device keeps running regardless.
//--------------------------------------------------

// Connect WiFi and configure the MQTT client. Call once from setup().
void initTelemetry();

// Service the MQTT client and reconnect WiFi/MQTT if needed.
// Call every loop tick.
void updateTelemetry();

// Serialize a SystemReport to JSON and publish it. No-op if offline.
void publishReport(const SystemReport& report);

// True when WiFi + MQTT are both connected.
bool telemetryOnline();

#endif
