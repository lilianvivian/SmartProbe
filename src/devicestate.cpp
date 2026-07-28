#include "devicestate.h"

static DeviceState currentState = DEVICE_IDLE;

void setDeviceState(DeviceState state)
{
    currentState = state;
}

DeviceState getDeviceState()
{
    return currentState;
}