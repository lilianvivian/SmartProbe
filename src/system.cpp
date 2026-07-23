#include <Arduino.h>
#include "system.h"

#include "guardian.h"
#include "tinyml.h"
#include "display.h"
#include "sensors.h"
#include "actions.h"
#include "leds.h"
#include "buzzer.h"
#include "communication.h"
#include "fan.h"
#include "systemreport.h"
#include "gsm.h"
#include "buttons.h"

void updateSystem()
{
    //--------------------------------------------------
    // Read Environmental Data
    //--------------------------------------------------
    report.temperature = getTemperature();
    report.humidity = getHumidity();
    report.pressure = getPressure();   // Reserved for future use
    report.voc = getVOC();


    //--------------------------------------------------
// Sensor Health Check
//--------------------------------------------------

if (isnan(report.temperature) || isnan(report.humidity))
{
    Serial.println();
    Serial.println("***** SENSOR FAILURE *****");

    showDashboard(
        0,
        0,
        0,
        "N/A",
        "ERROR",
        "CHECK",
        0
    );

    updateLEDs(false, true, false);   // Yellow LED

    updateBuzzer(false);

    updateButtons();
    if (inspectPressed())
    {
        Serial.println("Manual Inspection Requested");
    }

    static bool manualMode = false;

    if(modePressed())
    {
        manualMode = !manualMode;

        if(manualMode)
            Serial.println("MANUAL MODE");
        else
            Serial.println("AUTO MODE");
    }

    return;
}

    //--------------------------------------------------
    // Guardian Evaluation
    //--------------------------------------------------
    report.guardian = evaluateStorage(
        report.temperature,
        report.humidity,
        report.voc
    );

    //--------------------------------------------------
    // TinyML Prediction
    //--------------------------------------------------
    report.prediction = predictInspection(
        report.temperature,
        report.humidity,
        report.voc
    );

    //--------------------------------------------------
    // Update OLED Dashboard
    //--------------------------------------------------
    showDashboard(
        report.temperature,
        report.humidity,
        report.voc,
        gasLevel(report.voc),
        statusToString(report.guardian.status),
        trendToLCDString(report.prediction.trend),
        report.prediction.nextInspectionDays
    );

   //-------------------------------------------------
    // Execute Guardian Recommendations
    //--------------------------------------------------
    updateFan(report.guardian.runFan);

    updateLEDs(
        report.guardian.greenLed,
        report.guardian.yellowLed,
        report.guardian.redLed
    );

    updateBuzzer(report.guardian.soundAlarm);

    //--------------------------------------------------
    // Communication
    //--------------------------------------------------
    updateCommunication();

    //--------------------------------------------------
    // Execute Recommended Actions
    //--------------------------------------------------
    executeActions(report.guardian.status);

    //--------------------------------------------------
    // Debug Output (Temporary)
    //--------------------------------------------------
    Serial.println();
Serial.println("========== SMART PROBE ==========");

Serial.print("Temperature : ");
Serial.print(report.temperature,1);
Serial.println(" C");

Serial.print("Humidity    : ");
Serial.print(report.humidity,1);
Serial.println(" %");

Serial.print("VOC         : ");
Serial.println(report.voc);

Serial.println();

Serial.print("Status      : ");
Serial.println(statusToString(report.guardian.status));

Serial.print("Cause       : ");
Serial.println(causeToString(report.guardian.cause));

Serial.print("Action      : ");
Serial.println(report.guardian.recommendation);

Serial.println();

Serial.print("Trend       : ");
Serial.println(trendToString(report.prediction.trend));

Serial.print("Inspection  : ");
Serial.print(report.prediction.nextInspectionDays);
Serial.println(" days");

Serial.print("Confidence  : ");
Serial.print(report.prediction.confidence * 100,0);
Serial.println("%");

Serial.println("===============================");
Serial.println();
}