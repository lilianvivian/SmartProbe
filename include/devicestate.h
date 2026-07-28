#pragma once

enum DeviceState
{
    DEVICE_OFF,
    DEVICE_IDLE,
    DEVICE_INSPECTING,
    DEVICE_MENU,
    DEVICE_SLEEP,
    
};

void setDeviceState(DeviceState state);

DeviceState getDeviceState();