#include <Arduino.h>
#include "actions.h"

void executeActions(GrainStatus status)
{
    switch (status)
    {
        case SAFE:
            Serial.println("Storage conditions are safe. No action required.");
            break;
        case WARNING:
            Serial.println("Warning: Storage conditions are deteriorating.");
            break;
        case CRITICAL:
            Serial.println("Critical: Immediate action required to protect the stored grain!");
            break;
    }
}