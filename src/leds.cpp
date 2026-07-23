#include <Arduino.h>
#include "leds.h"

#define GREEN_LED   15
#define YELLOW_LED  16
#define RED_LED     17

void initLEDs()
{
    pinMode(GREEN_LED, OUTPUT);
    pinMode(YELLOW_LED, OUTPUT);
    pinMode(RED_LED, OUTPUT);

    digitalWrite(GREEN_LED, LOW);
    digitalWrite(YELLOW_LED, LOW);
    digitalWrite(RED_LED, LOW);
}

void greenLed(bool state)
{
    digitalWrite(GREEN_LED, state ? HIGH : LOW);
}

void yellowLed(bool state)
{
    digitalWrite(YELLOW_LED, state ? HIGH : LOW);
}

void redLed(bool state)
{
    digitalWrite(RED_LED, state ? HIGH : LOW);
}

void updateLEDs(
    bool green,
    bool yellow,
    bool red)
{
    greenLed(green);
    yellowLed(yellow);
    redLed(red);
}