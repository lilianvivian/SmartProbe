#include <Arduino.h>
#include "communication.h"
#include "commands.h"
#include "config.h"

//--------------------------------------------------
// Smart Probe Identification
//--------------------------------------------------
// Single definition for the whole firmware (declared extern in config.h).
const String PROBE_ID = "SP001";

//--------------------------------------------------
// Communication Initialization
//--------------------------------------------------
void initCommunication()
{
    Serial.begin(115200);

    Serial.println();
    Serial.println("=================================");
    Serial.println(" SMART PROBE Communication Ready");
    Serial.print(" Probe ID : ");
    Serial.println(PROBE_ID);
    Serial.println("=================================");
    Serial.println();
}

//--------------------------------------------------
// Communication Manager
//--------------------------------------------------
void updateCommunication()
{
    // -------------------------------------------------
    // Wokwi / USB Serial Testing
    // -------------------------------------------------
    // Later this section will be replaced with
    // incoming SMS messages from the GSM module.
    // -------------------------------------------------

    if (!Serial.available())
        return;

    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.length() == 0)
        return;

    Serial.println();
    Serial.println("------------ Incoming Command ------------");
    Serial.print("Probe : ");
    Serial.println(PROBE_ID);

    Serial.print("Message : ");
    Serial.println(command);

    Serial.println("------------------------------------------");

    // Process the received command
    processCommand(command);

    Serial.println("------------------------------------------");
    Serial.println();
}