#pragma once

enum OperatingMode
{
    MODE_AUTO,
    MODE_OVERRIDE,
};

void setOperatingMode(OperatingMode mode);

OperatingMode getOperatingMode();

void toggleOperatingMode();