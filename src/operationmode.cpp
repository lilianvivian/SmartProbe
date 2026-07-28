#include "operatingmode.h"

static OperatingMode currentMode = MODE_AUTO;

void setOperatingMode(OperatingMode mode)
{
    currentMode = mode;
}

OperatingMode getOperatingMode()
{
    return currentMode;
}

void toggleOperatingMode()
{
    if(currentMode == MODE_AUTO)
        currentMode = MODE_OVERRIDE;
    else
        currentMode = MODE_AUTO;
}