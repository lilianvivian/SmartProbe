#include "events.h"

static SystemEvent pendingEvent = EVENT_NONE;

void postEvent(SystemEvent event)
{
    pendingEvent = event;
}

SystemEvent getNextEvent()
{
    SystemEvent event = pendingEvent;
    pendingEvent = EVENT_NONE;
    return event;
}