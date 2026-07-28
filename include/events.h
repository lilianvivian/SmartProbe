#pragma once

enum SystemEvent
{
    EVENT_NONE,

    EVENT_POWER_SHORT,
    EVENT_POWER_LONG,

    EVENT_MODE_SHORT,
    EVENT_MODE_LONG,

    EVENT_INSPECT_SHORT,
    EVENT_INSPECT_LONG,
};

void postEvent(SystemEvent event);


SystemEvent getNextEvent();