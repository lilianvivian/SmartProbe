#include <Arduino.h>
#include "commands.h"
#include "systemreport.h"
#include "config.h"

// PROBE_ID is defined once in communication.cpp (declared extern in config.h).

//--------------------------------------------------
// Process Incoming Command
//--------------------------------------------------
void processCommand(const String& command)
{
    String cmd = command;
    cmd.trim();
    cmd.toUpperCase();

    Serial.println();
    Serial.println("========== COMMAND ==========");
    Serial.print("Probe ID : ");
    Serial.println(PROBE_ID);
    Serial.print("Command  : ");
    Serial.println(cmd);
    Serial.println("=============================");

    //--------------------------------------------------
    // STATUS
    //--------------------------------------------------
    if (cmd == "STATUS")
    {
        Serial.println("------ CURRENT STATUS ------");

        Serial.print("Status : ");
        Serial.println(statusToString(report.guardian.status));

        Serial.print("Cause  : ");
        Serial.println(causeToString(report.guardian.cause));

        Serial.println("----------------------------");
    }

    //--------------------------------------------------
    // INSPECT
    //--------------------------------------------------
    else if (cmd == "INSPECT")
    {
        Serial.println("------ AI INSPECTION ------");

        Serial.print("Trend : ");
        Serial.println(trendToString(report.prediction.trend));

        Serial.print("Next Inspection : ");
        Serial.print(report.prediction.nextInspectionDays);
        Serial.println(" days");

        Serial.print("Confidence : ");
        Serial.print(report.prediction.confidence * 100, 1);
        Serial.println("%");

        Serial.println("---------------------------");
    }

    //--------------------------------------------------
    // GUARDIAN
    //--------------------------------------------------
    else if (cmd == "GUARDIAN")
    {
        Serial.println("------ GUARDIAN ------");

        Serial.print("Storage Status : ");
        Serial.println(statusToString(report.guardian.status));

        Serial.print("Primary Cause  : ");
        Serial.println(causeToString(report.guardian.cause));

        Serial.print("Fan : ");
        Serial.println(report.guardian.runFan ? "ON" : "OFF");

        Serial.print("Alarm : ");
        Serial.println(report.guardian.soundAlarm ? "ON" : "OFF");

        Serial.println("----------------------");
    }

    //--------------------------------------------------
    // REPORT
    //--------------------------------------------------
    else if (cmd == "REPORT")
    {
        Serial.println();
        Serial.println("========== SMART PROBE REPORT ==========");

        Serial.print("Probe ID : ");
        Serial.println(PROBE_ID);

        Serial.println();

        Serial.print("Temperature : ");
        Serial.print(report.temperature,1);
        Serial.println(" C");

        Serial.print("Humidity    : ");
        Serial.print(report.humidity,1);
        Serial.println(" %");

        Serial.print("Pressure    : ");
        Serial.print(report.pressure,1);
        Serial.println(" hPa");

        Serial.print("VOC         : ");
        Serial.println(report.voc,0);

        Serial.println();

        Serial.print("Status : ");
        Serial.println(statusToString(report.guardian.status));

        Serial.print("Cause  : ");
        Serial.println(causeToString(report.guardian.cause));

        Serial.println();

        Serial.print("AI Trend : ");
        Serial.println(trendToString(report.prediction.trend));

        Serial.print("Next Inspection : ");
        Serial.print(report.prediction.nextInspectionDays);
        Serial.println(" days");

        Serial.print("Confidence : ");
        Serial.print(report.prediction.confidence * 100,1);
        Serial.println("%");

       Serial.println();
Serial.println("Recommended Action");

//--------------------------------------------------
// SAFE
//--------------------------------------------------
if(report.guardian.status == SAFE)
{
    Serial.println("- Grain is in good storage condition.");
    Serial.print("- Next inspection in ");
    Serial.print(report.prediction.nextInspectionDays);
    Serial.println(" days.");
}

//--------------------------------------------------
// WARNING
//--------------------------------------------------
else if(report.guardian.status == WARNING)
{
    switch(report.guardian.cause)
    {
        case HUMIDITY:
            Serial.println("- Humidity is increasing.");
            Serial.println("- Improve ventilation.");
            Serial.println("- Check bag sealing.");
            break;

        case TEMPERATURE:
            Serial.println("- Temperature is rising.");
            Serial.println("- Move bag to a cooler location.");
            Serial.println("- Reduce direct sunlight exposure.");
            break;

        case VOC:
            Serial.println("- VOC levels are increasing.");
            Serial.println("- Early fungal activity suspected.");
            Serial.println("- Inspect grain soon.");
            break;

        default:
            Serial.println("- Continue monitoring.");
    }

    Serial.print("- Inspect again in ");
    Serial.print(report.prediction.nextInspectionDays);
    Serial.println(" days.");
}

//--------------------------------------------------
// CRITICAL
//--------------------------------------------------
else
{
    switch(report.guardian.cause)
    {
        case HUMIDITY:
            Serial.println("- Excess moisture detected.");
            Serial.println("- Activate ventilation.");
            Serial.println("- Dry grain immediately.");
            break;

        case TEMPERATURE:
            Serial.println("- Dangerous temperature detected.");
            Serial.println("- Relocate grain immediately.");
            break;

        case VOC:
            Serial.println("- High VOC concentration detected.");
            Serial.println("- Possible fungal growth.");
            Serial.println("- Isolate and inspect this bag immediately.");
            break;

        default:
            Serial.println("- Immediate inspection required.");
    }

    if(report.guardian.runFan)
        Serial.println("- Guardian recommends turning the fan ON.");

    if(report.guardian.soundAlarm)
        Serial.println("- Alarm has been activated.");
}

        Serial.println("========================================");
    }

    //--------------------------------------------------
    // FAN ON
    //--------------------------------------------------
    else if (cmd == "FAN ON")
    {
        Serial.println("Manual Override Enabled");
        Serial.println("Fan forced ON.");

        // TODO:
        // manualOverride = true;
        // fanOn();
    }

    //--------------------------------------------------
    // FAN OFF
    //--------------------------------------------------
    else if (cmd == "FAN OFF")
    {
        Serial.println("Manual Override Disabled");
        Serial.println("Returning control to Guardian.");

        // TODO:
        // manualOverride = false;
    }

    //--------------------------------------------------
    // RESET
    //--------------------------------------------------
    else if (cmd == "RESET")
    {
        Serial.println("Resetting Smart Probe...");

        // TODO
        // Reset TinyML history
        // Reset Guardian
        // Register new grain bag
        // Clear inspection schedule

        Serial.println("System ready for a new grain bag.");
    }

    //--------------------------------------------------
    // HELP
    //--------------------------------------------------
    else if (cmd == "HELP")
    {
        Serial.println();
        Serial.println("======= SMART PROBE COMMANDS =======");
        Serial.println("STATUS    - Current storage status");
        Serial.println("REPORT    - Complete Smart Probe report");
        Serial.println("INSPECT   - AI inspection prediction");
        Serial.println("GUARDIAN  - Guardian decision");
        Serial.println("FAN ON    - Manual fan override");
        Serial.println("FAN OFF   - Return fan to Guardian");
        Serial.println("RESET     - Start monitoring new bag");
        Serial.println("HELP      - Show commands");
        Serial.println("====================================");
    }

    //--------------------------------------------------
    // UNKNOWN COMMAND
    //--------------------------------------------------
    else
    {
        Serial.println("[ERROR]");
        Serial.println("Unknown command.");
        Serial.println("Type HELP to view available commands.");
    }

    Serial.println();
}