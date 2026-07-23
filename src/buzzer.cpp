#include <Arduino.h>
#include "buzzer.h"

#define BUZZER_PIN 18

bool buzzerState = false;
unsigned long buzzerPreviousMillis = 0;

//--------------------------------------------------

void initBuzzer()
{
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
}

//--------------------------------------------------

void buzzerOn()
{
    unsigned long currentMillis = millis();

    // Non-blocking intermittent alarm
    if (currentMillis - buzzerPreviousMillis >= 250)
    {
        buzzerPreviousMillis = currentMillis;

        buzzerState = !buzzerState;

        digitalWrite(BUZZER_PIN, buzzerState);
    }
}

//--------------------------------------------------

void buzzerOff()
{
    digitalWrite(BUZZER_PIN, LOW);
    buzzerState = false;
}

//--------------------------------------------------

void updateBuzzer(bool enable)
{
    if (enable)
    {
        buzzerOn();
    }
    else
    {
        buzzerOff();
    }
}