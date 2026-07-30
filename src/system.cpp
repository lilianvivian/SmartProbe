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
#include "events.h"
#include "devicestate.h"
#include "operatingmode.h"
#include "config.h"

static bool criticalAlertSent = false;
static unsigned long lastSMSAttempt = 0;
const unsigned long smsRetryInterval = 60000; // 1 minute

// Automatic inspection timer
static unsigned long lastInspection = 0;

// Inspection interval (milliseconds)
// 30000 = 30 seconds (good for testing)
static const unsigned long inspectionInterval = 30000;

void performInspection();
void handleOffState(SystemEvent event);
void handleIdleState(SystemEvent event);
void handleInspectionState(SystemEvent event);
void handleMenuState(SystemEvent event);
void handleSleepState(SystemEvent event);

void updateSystem()
{
    
    updateButtons();
    SystemEvent event = getNextEvent();


    switch(getDeviceState())
    {
        case DEVICE_OFF:
            handleOffState(event);
            break;


        case DEVICE_IDLE:
            handleIdleState(event);
            break;

        case DEVICE_INSPECTING:
            handleInspectionState(event);
            break;

        case DEVICE_MENU:
            handleMenuState(event);
            break;

        case DEVICE_SLEEP:
            handleSleepState(event);
            break;
    }
}

    
   
void handleOffState(SystemEvent event)
{
    switch(event)
    {
        case EVENT_POWER_LONG:
        Serial.println("Booting Smart Probe...");
            setDeviceState(DEVICE_IDLE);
            break;

        default:
            break;
    }
}

void handleIdleState(SystemEvent event)
{

    if(millis() - lastInspection >= inspectionInterval)
    {
        lastInspection = millis();
        setDeviceState(DEVICE_INSPECTING);
        return;
    }
    switch(event)
    {
        case EVENT_INSPECT_SHORT:
            Serial.println("Immediate Inspection Requested");
            setDeviceState(DEVICE_INSPECTING);
            return;
            break;

        case EVENT_INSPECT_LONG:
            Serial.println("Sensor Calibration");
            break;

        case EVENT_MODE_SHORT:
            toggleOperatingMode();

            if(getOperatingMode() == MODE_AUTO)
                Serial.println("AUTO Mode");
            else
                Serial.println("OVERRIDE Mode");
            break;

        case EVENT_MODE_LONG:
            Serial.println("Opening Settings");
            setDeviceState(DEVICE_MENU);
            return;
            break;

        case EVENT_POWER_LONG:
            Serial.println("Entering Sleep");
            setDeviceState(DEVICE_SLEEP);
            return;
            break;

        default:
            break;
    }
}

void performInspection()
{
    //--------------------------------------------------
    // Read Sensors
    //--------------------------------------------------
    report.temperature = getTemperature();
    report.humidity = getHumidity();
    report.pressure = getPressure();
    report.voc = getVOC();

    //--------------------------------------------------
    // Sensor Health
    //--------------------------------------------------
    if (isnan(report.temperature) || isnan(report.humidity))
    {
        Serial.println("Sensor Failure");

        showDashboard(
            0,
            0,
            0,
            "N/A",
            "ERROR",
            "CHECK",
            0
        );

        updateLEDs(false, true, false);
        updateBuzzer(false);

        return;
    }

    //--------------------------------------------------
    // Guardian
    //--------------------------------------------------
    report.guardian = evaluateStorage(
        report.temperature,
        report.humidity,
        report.voc);

    //--------------------------------------------------
// GSM Alert
//--------------------------------------------------
if (report.guardian.sendSMS && !criticalAlertSent && millis() - lastSMSAttempt >= smsRetryInterval)
{
    lastSMSAttempt = millis();

    String message;

    message += "SMART PROBE ALERT\n\n";

    message += "Critical grain storage conditions detected.\n\n";

    message += "Temperature: ";
    message += String(report.temperature, 1);
    message += " C\n";

    message += "Humidity: ";
    message += String(report.humidity, 1);
    message += " %\n";

    message += "VOC: ";
    message += String(report.voc, 1);
    message += "\n\n";

    message += "Recommendation: ";
    message += report.guardian.recommendation;

    if(sendSMS(FARMER_PHONE, message))
    {
        Serial.println("Critical SMS sent.");
        criticalAlertSent = true;
    }
    else
    {
        Serial.println("SMS sending failed.");
    }
}

if(report.guardian.status != CRITICAL)
{
    criticalAlertSent = false;
}

    //--------------------------------------------------
    // TinyML
    //--------------------------------------------------
    report.prediction = predictInspection(
        report.temperature,
        report.humidity,
        report.voc);

    //--------------------------------------------------
    // OLED
    //--------------------------------------------------
    showDashboard(
        report.temperature,
        report.humidity,
        report.voc,
        gasLevel(report.voc),
        statusToString(report.guardian.status),
        trendToLCDString(report.prediction.trend),
        report.prediction.nextInspectionDays);

    //--------------------------------------------------
    // Outputs
    //--------------------------------------------------
    if(getOperatingMode() == MODE_AUTO){
       
    updateFan(report.guardian.runFan);

    updateLEDs(
        report.guardian.greenLed,
        report.guardian.yellowLed,
        report.guardian.redLed);

    updateBuzzer(report.guardian.soundAlarm);

    updateCommunication();

    executeActions(report.guardian.status);
    }
    else{
    // Override mode:
    // Do NOT automatically control fan or pump.
    // Still show recommendations on the display.
    }
}


void handleInspectionState(SystemEvent event)
{
    (void)event;

    Serial.println("Inspection Running...");

    performInspection();
    Serial.println("Inspection Complete");

    setDeviceState(DEVICE_IDLE);
}

void handleMenuState(SystemEvent event)
{
    switch(event)
    {
        case EVENT_MODE_LONG:
            Serial.println("Closing Menu");
            setDeviceState(DEVICE_IDLE);
            break;

        default:
            break;
    }
}

void handleSleepState(SystemEvent event)
{
    switch(event)
    {
        case EVENT_POWER_LONG:
            Serial.println("Waking Device");
            setDeviceState(DEVICE_IDLE);
            break;

        default:
            break;
    }
}