#include "guardian.h"

GuardianDecision evaluateStorage(
    float temperature,
    float humidity,
    float voc)
{
    //--------------------------------------------------
    // Individual Risk Scores
    //--------------------------------------------------
    int tempRisk = 0;
    int humidityRisk = 0;
    int vocRisk = 0;

    //--------------------------------------------------
    // Temperature Assessment
    //--------------------------------------------------
    if (temperature >= 38.0f)
        tempRisk = 30;
    else if (temperature >= 33.0f)
        tempRisk = 20;
    else if (temperature >= 30.0f)
        tempRisk = 10;

    //--------------------------------------------------
    // Humidity Assessment
    //--------------------------------------------------
    if (humidity >= 75.0f)
        humidityRisk = 35;
    else if (humidity >= 68.0f)
        humidityRisk = 20;
    else if (humidity >= 60.0f)
        humidityRisk = 10;

    //--------------------------------------------------
    // VOC Assessment
    //--------------------------------------------------
    if (voc >= 500.0f)
        vocRisk = 40;
    else if (voc >= 350.0f)
        vocRisk = 25;
    else if (voc >= 220.0f)
        vocRisk = 10;
    else
        vocRisk = 0;

    //--------------------------------------------------
    // Overall Risk
    //--------------------------------------------------
    int totalRisk = tempRisk + humidityRisk + vocRisk;

    GuardianDecision decision;

    //--------------------------------------------------
    // Storage Status
    //--------------------------------------------------
    if (totalRisk < 35)
        decision.status = SAFE;

    else if (totalRisk < 70)
        decision.status = WARNING;

    else
        decision.status = CRITICAL;

    //--------------------------------------------------
// Determine Main Cause
//--------------------------------------------------

if (totalRisk == 0)
{
    decision.cause = NONE;
}
else if (humidityRisk >= tempRisk &&
         humidityRisk >= vocRisk)
{
    decision.cause = HUMIDITY;
}
else if (tempRisk >= humidityRisk &&
         tempRisk >= vocRisk)
{
    decision.cause = TEMPERATURE;
}
else
{
    decision.cause = VOC;
}

//--------------------------------------------------
// Recommendation
//--------------------------------------------------

if (decision.status == SAFE)
{
    decision.recommendation = "Storage OK";
}
else
{
    switch (decision.cause)
    {
        case TEMPERATURE:
            decision.recommendation = "Cool Grain";
            break;

        case HUMIDITY:
            decision.recommendation = "Dry Grain";
            break;

        case VOC:
            decision.recommendation = "Check Grain";
            break;

        default:
            decision.recommendation = "Inspect";
            break;
    }
}

    //--------------------------------------------------
    // Guardian Recommendations
    //--------------------------------------------------

    // Fan Recommendation
    decision.runFan = (
        decision.status != SAFE && 
        decision.cause == HUMIDITY && 
        humidity >= 75.0f
    );

    if (decision.cause == HUMIDITY &&
        humidity >= 75.0f)
    {
        decision.runFan = true;
    }

    // Alarm Recommendation
    decision.soundAlarm = (decision.status == CRITICAL);

    //--------------------------------------------------
    // LED Recommendations
    //--------------------------------------------------
    decision.greenLed = false;
    decision.yellowLed = false;
    decision.redLed = false;

    switch (decision.status)
    {
        case SAFE:
            decision.greenLed = true;
            break;

        case WARNING:
            decision.yellowLed = true;
            break;

        case CRITICAL:
            decision.redLed = true;
            break;
    }

    return decision;
}

const char* statusToString(GrainStatus status)
{
    switch (status)
    {
        case SAFE:
            return "SAFE";

        case WARNING:
            return "WARNING";

        case CRITICAL:
            return "CRITICAL";

        default:
            return "UNKNOWN";
    }
}

const char* causeToString(GrainCause cause)
{
    switch (cause)
    {
        case TEMPERATURE:
            return "TEMPERATURE";

        case HUMIDITY:
            return "HUMIDITY";

        case VOC:
            return "VOC";

        default:
            return "NONE";
    }
}