#include <Arduino.h>
#include <DHT.h>
#include "sensors.h"

#define DHTPIN 4
#define DHTTYPE DHT11
#define MQ2_PIN 6

DHT dht(DHTPIN, DHTTYPE);
int mq2Pin = MQ2_PIN;

void initSensors()
{
    dht.begin();
    pinMode(mq2Pin, INPUT);
}

float getTemperature()
{
    float value = dht.readTemperature();

    if (isnan(value))
        return NAN;

    return value;
}

float getHumidity()
{
    float value = dht.readHumidity();

    if (isnan(value))
        return NAN;

    return value;
}

// Temporary values until BME688 arrives

float getPressure()
{
    return 1013.25;
}

float getVOC()
{
long total = 0;

    // Average 20 readings for a more stable value
    for (int i = 0; i < 20; i++)
    {
        total += analogRead(mq2Pin);
        delay(10);
    }

    return total / 20.0;
}

//--------------------------------------------------
// Gas Classification
//--------------------------------------------------

const char* gasLevel(float voc)
{
    if (voc < 220)
        return "LOW";

    else if (voc < 400)
        return "MEDIUM";

    else
        return "HIGH";
}