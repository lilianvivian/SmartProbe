#include <Arduino.h>
#include "fan.h"
#include "pins.h"

// FAN_PIN is -1 while no fan is wired, so every entry point bails out rather
// than driving an invalid GPIO once a second.

void initFan()
{
    if(FAN_PIN < 0)
        return;

    pinMode(FAN_PIN, OUTPUT);
    digitalWrite(FAN_PIN, LOW);
}

void fanOn()
{
    if(FAN_PIN < 0)
        return;

    digitalWrite(FAN_PIN, HIGH);
}

void fanOff()
{
    if(FAN_PIN < 0)
        return;

    digitalWrite(FAN_PIN, LOW);
}

void updateFan(bool enable)
{
    if(enable)
        fanOn();
    else
        fanOff();
}