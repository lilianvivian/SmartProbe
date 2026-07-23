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

void setup()
{
    Serial.begin(115200);

    // Initialize Hardware
    initDisplay();
    initLEDs();
    initBuzzer();
    initCommunication();
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

    updateProgress("GSM Module");

if(checkModule())
{
    Serial.println("GSM Ready");
}
else
{
    Serial.println("GSM Not Detected");
}

delay(300);

    showReady();
}

void loop()
{
    updateSystem();

    delay(1000);
}