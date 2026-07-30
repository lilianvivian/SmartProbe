#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#include "telemetry.h"
#include "config.h"
#include "secrets.h"

#include "guardian.h"   // statusToString, causeToString
#include "tinyml.h"     // trendToString
#include "sensors.h"    // gasLevel
#include "operatingmode.h"

//--------------------------------------------------
// MQTT client
//--------------------------------------------------
static WiFiClient   wifiClient;
static PubSubClient mqtt(wifiClient);

// Cached per-probe topics (built once from PROBE_ID)
static String telemetryTopic;
static String statusTopic;

// Non-blocking reconnect pacing
static unsigned long lastWifiAttempt = 0;
static unsigned long lastMqttAttempt = 0;
static const unsigned long RECONNECT_INTERVAL = 5000; // ms

//--------------------------------------------------
// Initialization
//--------------------------------------------------
void initTelemetry()
{
    telemetryTopic = String(MQTT_TOPIC_PREFIX) + "/" + PROBE_ID + "/telemetry";
    statusTopic    = String(MQTT_TOPIC_PREFIX) + "/" + PROBE_ID + "/status";

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    mqtt.setServer(MQTT_HOST, MQTT_PORT);
    // Default PubSubClient buffer (256 B) is a little tight for our payload.
    mqtt.setBufferSize(512);

    Serial.print("Telemetry: connecting to WiFi \"");
    Serial.print(WIFI_SSID);
    Serial.println("\"...");
}

//--------------------------------------------------
// Connection helpers (non-blocking, paced by millis)
//--------------------------------------------------
static void ensureWifi()
{
    if (WiFi.status() == WL_CONNECTED)
        return;

    if (millis() - lastWifiAttempt < RECONNECT_INTERVAL)
        return;

    lastWifiAttempt = millis();
    Serial.println("Telemetry: WiFi down, reconnecting...");
    WiFi.reconnect();
}

static void ensureMqtt()
{
    if (WiFi.status() != WL_CONNECTED || mqtt.connected())
        return;

    if (millis() - lastMqttAttempt < RECONNECT_INTERVAL)
        return;

    lastMqttAttempt = millis();

    Serial.print("Telemetry: connecting to MQTT ");
    Serial.print(MQTT_HOST);
    Serial.print(":");
    Serial.print(MQTT_PORT);
    Serial.print(" ... ");

    // Last-will marks the probe offline if it drops unexpectedly.
    bool ok;
    const char* clientId = PROBE_ID.c_str();

    if (strlen(MQTT_USER) > 0)
        ok = mqtt.connect(clientId, MQTT_USER, MQTT_PASS,
                          statusTopic.c_str(), 0, true, "{\"online\":false}");
    else
        ok = mqtt.connect(clientId, statusTopic.c_str(), 0, true,
                          "{\"online\":false}");

    if (ok)
    {
        Serial.println("connected");
        // Retained birth message so a freshly opened dashboard sees us online.
        mqtt.publish(statusTopic.c_str(), "{\"online\":true}", true);
    }
    else
    {
        Serial.print("failed, rc=");
        Serial.println(mqtt.state());
    }
}

//--------------------------------------------------
// Loop service
//--------------------------------------------------
void updateTelemetry()
{
    ensureWifi();
    ensureMqtt();
    mqtt.loop();
}

bool telemetryOnline()
{
    return WiFi.status() == WL_CONNECTED && mqtt.connected();
}

//--------------------------------------------------
// Publish one inspection
//--------------------------------------------------
void publishReport(const SystemReport& report)
{
    if (!telemetryOnline())
    {
        Serial.println("Telemetry: offline, skipping publish");
        return;
    }

    JsonDocument doc;

    doc["id"]                 = PROBE_ID;
    doc["t"]                  = millis();
    doc["temperature"]        = report.temperature;
    doc["humidity"]           = report.humidity;
    doc["pressure"]           = report.pressure;
    doc["voc"]                = report.voc;
    doc["gas"]                = gasLevel(report.voc);
    doc["status"]             = statusToString(report.guardian.status);
    doc["cause"]              = causeToString(report.guardian.cause);
    doc["recommendation"]     = report.guardian.recommendation;
    doc["runFan"]             = report.guardian.runFan;
    doc["soundAlarm"]         = report.guardian.soundAlarm;
    doc["trend"]              = trendToString(report.prediction.trend);
    doc["nextInspectionDays"] = report.prediction.nextInspectionDays;
    doc["confidence"]         = report.prediction.confidence;
    doc["mode"]               = (getOperatingMode() == MODE_AUTO) ? "AUTO" : "OVERRIDE";

    char payload[512];
    size_t n = serializeJson(doc, payload, sizeof(payload));

    if (mqtt.publish(telemetryTopic.c_str(), payload, n))
    {
        Serial.print("Telemetry: published ");
        Serial.print(telemetryTopic);
        Serial.print(" ");
        Serial.println(payload);
    }
    else
    {
        Serial.println("Telemetry: publish failed");
    }
}
