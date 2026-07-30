#include <Arduino.h>

#include "display.h"
#include "sensors.h"
#include "guardian.h"
#include "actions.h"
#include "leds.h"
#include "buzzer.h"
#include "system.h"
#include "communication.h"
#include "fan.h"
#include "gsm.h"
#include "buttons.h"
#include "telemetry.h"

void setup()
{
    Serial.begin(115200);

    // Initialize Hardware
    initDisplay();
    initLEDs();
    initBuzzer();
    initCommunication();
    initTelemetry();
    initGSM();
    initFan();
    initButtons();

    initSensors();

    // Boot Sequence
    showBootScreen();
    delay(2000);

    showInitializing();

    updateProgress("Display");
    delay(300);

    updateProgress("Sensors");
    delay(300);

    updateProgress("AI Engine");
    delay(300);

    updateProgress("WiFi");
    delay(300);

    updateProgress("GSM Module");

    delay(300);

    showReady();
}

void loop()
{
    updateSystem();
    updateButtons();
    updateTelemetry();

    delay(1000);
}