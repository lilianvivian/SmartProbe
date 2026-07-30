#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

//--------------------------------------------------
// Probe Identity
//--------------------------------------------------
// Single source of truth. Defined once in communication.cpp.
extern const String PROBE_ID;

//--------------------------------------------------
// MQTT Topics
//--------------------------------------------------
// Per-probe topics: smartprobe/<PROBE_ID>/telemetry etc.
#define MQTT_TOPIC_PREFIX  "smartprobe"

#define TEMP_WARNING       32
#define TEMP_CRITICAL      38

#define HUM_WARNING        65
#define HUM_CRITICAL       75

#define VOC_WARNING        40
#define VOC_CRITICAL       80

const char* const FARMER_PHONE = "+254765156670";

#endif
