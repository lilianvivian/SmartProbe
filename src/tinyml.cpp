#include "tinyml.h"

void updateHistory(float temperature, float humidity, float voc);
float averageSlope(float history[]);
float averageValue(float history[]);

//--------------------------------------------------
// History Buffers
//--------------------------------------------------

float temperatureHistory[HISTORY_SIZE] = {0};
float humidityHistory[HISTORY_SIZE] = {0};
float vocHistory[HISTORY_SIZE] = {0};

bool historyFilled = false;
int historyIndex = 0;

//--------------------------------------------------
// Prediction Thresholds
//--------------------------------------------------

constexpr float TEMP_LIMIT = 33.0f;
constexpr float HUMIDITY_LIMIT = 70.0f;
constexpr float VOC_LIMIT = 350.0f;

constexpr float TEMP_SLOPE_LIMIT = 0.5f;
constexpr float HUMIDITY_SLOPE_LIMIT = 0.8f;
constexpr float VOC_SLOPE_LIMIT = 8.0f;

//--------------------------------------------------
// Initialize TinyML
//--------------------------------------------------

void initTinyML()
{
    for (int i = 0; i < HISTORY_SIZE; i++)
    {
        temperatureHistory[i] = 0;
        humidityHistory[i] = 0;
        vocHistory[i] = 0;
    }

    historyIndex = 0;
    historyFilled = false;
}

//--------------------------------------------------
// Store Latest Reading
//--------------------------------------------------

void updateHistory(float temperature, float humidity, float voc)
{
    temperatureHistory[historyIndex] = temperature;
    humidityHistory[historyIndex] = humidity;
    vocHistory[historyIndex] = voc;

    historyIndex++;

    if (historyIndex >= HISTORY_SIZE)
    {
        historyIndex = 0;
        historyFilled = true;
    }
}

//--------------------------------------------------
// Average Rate of Change
//--------------------------------------------------

float averageSlope(float history[])
{
    int count = historyFilled ? HISTORY_SIZE : historyIndex;

    if (count < 2)
        return 0;

    float total = 0;

    for (int i = 1; i < count; i++)
    {
        total += history[i] - history[i - 1];
    }

    return total / (count - 1);
}

//--------------------------------------------------
// Average Value
//--------------------------------------------------

float averageValue(float history[])
{
    int count = historyFilled ? HISTORY_SIZE : historyIndex;

    if (count == 0)
        return 0;

    float total = 0;

    for (int i = 0; i < count; i++)
        total += history[i];

    return total / count;
}

//--------------------------------------------------
// Prediction Engine
//--------------------------------------------------

Prediction predictInspection(
    float temperature,
    float humidity,
    float voc)
{
    updateHistory(temperature, humidity, voc);

    Prediction prediction;

    float tempSlope = averageSlope(temperatureHistory);
    float humiditySlope = averageSlope(humidityHistory);
    float vocSlope = averageSlope(vocHistory);

    float avgTemperature = averageValue(temperatureHistory);
    float avgHumidity = averageValue(humidityHistory);
    float avgVOC = averageValue(vocHistory);

    int deteriorationScore = 0;
    int evidenceCount = 0;

    //--------------------------------------------------
    // Trend Analysis
    //--------------------------------------------------

    if (tempSlope > TEMP_SLOPE_LIMIT)
    {
        deteriorationScore++;
        evidenceCount++;
    }

    if (humiditySlope > HUMIDITY_SLOPE_LIMIT)
    {
        deteriorationScore++;
        evidenceCount++;
    }

    if (vocSlope > VOC_SLOPE_LIMIT)
    {
        deteriorationScore += 2;
        evidenceCount++;
    }

    //--------------------------------------------------
    // Current Conditions
    //--------------------------------------------------

    // Temperature
    if (temperature >= TEMP_LIMIT)
    {
        deteriorationScore += 2;
        evidenceCount++;
    }
    else if (temperature > avgTemperature + 1.0f)
    {
        deteriorationScore++;
        evidenceCount++;
    }

    // Humidity
    if (humidity >= HUMIDITY_LIMIT)
    {
        deteriorationScore += 2;
        evidenceCount++;
    }
    else if (humidity > avgHumidity + 2.0f)
    {
        deteriorationScore++;
        evidenceCount++;
    }

    // VOC
    if (voc >= VOC_LIMIT)
    {
        deteriorationScore += 3;
        evidenceCount++;
    }
    else if (voc > avgVOC + 20.0f)
    {
        deteriorationScore += 2;
        evidenceCount++;
    }

    //--------------------------------------------------
    // Prediction
    //--------------------------------------------------

    if (deteriorationScore <= 2)
    {
        prediction.trend = STABLE;
        prediction.nextInspectionDays = 14;
    }
    else if (deteriorationScore <= 4)
    {
        prediction.trend = IMPROVING;
        prediction.nextInspectionDays = 7;
    }
    else if (deteriorationScore <= 7)
    {
        prediction.trend = SLOWLY_DETERIORATING;
        prediction.nextInspectionDays = 4;
    }
    else
    {
        prediction.trend = RAPIDLY_DETERIORATING;
        prediction.nextInspectionDays = 2;
    }

    //--------------------------------------------------
    // Dynamic Confidence
    //--------------------------------------------------

    const int MAX_EVIDENCE = 6;

    prediction.confidence =
        0.50f +
        0.49f *
        ((float)evidenceCount / MAX_EVIDENCE);

    if (prediction.confidence > 0.99f)
        prediction.confidence = 0.99f;

    return prediction;
}

//--------------------------------------------------
// Trend Strings
//--------------------------------------------------

const char* trendToString(Trend trend)
{
    switch (trend)
    {
        case STABLE:
            return "STABLE";

        case IMPROVING:
            return "IMPROVING";

        case SLOWLY_DETERIORATING:
            return "DETERIORATING";

        case RAPIDLY_DETERIORATING:
            return "RAPIDLY DETERIORATING";

        default:
            return "UNKNOWN";
    }
}

const char* trendToLCDString(Trend trend)
{
    switch (trend)
    {
        case STABLE:
            return "STABLE";

        case IMPROVING:
            return "GOOD";

        case SLOWLY_DETERIORATING:
            return "WATCH";

        case RAPIDLY_DETERIORATING:
            return "ALERT";

        default:
            return "---";
    }
}