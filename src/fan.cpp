#include <Arduino.h>
#include "fan.h"

#define FAN_PIN 4

void initFan()
{
    pinMode(FAN_PIN, OUTPUT);
    digitalWrite(FAN_PIN, LOW);
}

void fanOn()
{
    digitalWrite(FAN_PIN, HIGH);
}

void fanOff()
{
    digitalWrite(FAN_PIN, LOW);
}

void updateFan(bool enable)
{
    if(enable)
        fanOn();
    else
        fanOff();
}