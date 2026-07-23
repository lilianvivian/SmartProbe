#include <Arduino.h>
#include "buttons.h"
#include "pins.h"



bool inspectFlag = false;
bool modeFlag = false;
bool powerFlag = false;

void initButtons()
{
    pinMode(INSPECT_BUTTON, INPUT_PULLUP);
    pinMode(MODE_BUTTON, INPUT_PULLUP);
    pinMode(POWER_BUTTON, INPUT_PULLUP);
}

void updateButtons()
{
    static bool lastInspect = HIGH;
    static bool lastMode = HIGH;
    static bool lastPower = HIGH;

    bool inspect = digitalRead(INSPECT_BUTTON);
    bool mode = digitalRead(MODE_BUTTON);
    bool power = digitalRead(POWER_BUTTON);

    if(lastInspect == HIGH && inspect == LOW)
        inspectFlag = true;

    if(lastMode == HIGH && mode == LOW)
        modeFlag = true;

    if(lastPower == HIGH && power == LOW)
        powerFlag = true;

    lastInspect = inspect;
    lastMode = mode;
    lastPower = power;
}

bool inspectPressed()
{
    if(inspectFlag)
    {
        inspectFlag = false;
        return true;
    }

    return false;
}

bool modePressed()
{
    if(modeFlag)
    {
        modeFlag = false;
        return true;
    }

    return false;
}

bool powerPressed()
{
    if(powerFlag)
    {
        powerFlag = false;
        return true;
    }

    return false;
}