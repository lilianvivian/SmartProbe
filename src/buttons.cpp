#include <Arduino.h>
#include "buttons.h"
#include "pins.h"
#include "events.h"



const unsigned long debounceDelay = 30;
const unsigned long longPressTime = 2000;

struct ButtonState
{
    bool lastReading = HIGH;
    bool stableState = HIGH;

    unsigned long lastDebounceTime = 0;

    unsigned long pressedTime = 0;

    bool longReported = false;
};

ButtonState inspectButton;
ButtonState modeButton;
ButtonState powerButton;

void initButtons()
{
    pinMode(INSPECT_BUTTON, INPUT_PULLUP);
    pinMode(MODE_BUTTON, INPUT_PULLUP);
    pinMode(POWER_BUTTON, INPUT_PULLUP);
}

void processButton(
    int pin,
    ButtonState &button
    )
{
    bool reading = digitalRead(pin);

    if (reading != button.lastReading)
    {
        button.lastDebounceTime = millis();
    }

    if ((millis() - button.lastDebounceTime) > debounceDelay)
    {
        if (reading != button.stableState)
        {
            button.stableState = reading;

            // Button pressed
            if (button.stableState == LOW)
            {
                button.pressedTime = millis();
                button.longReported = false;
            }

            // Button released
            else
            {
                if (!button.longReported)
                {
                    switch (pin)
                {
                    case INSPECT_BUTTON:
                        postEvent(EVENT_INSPECT_SHORT);
                        break;

                    case MODE_BUTTON:
                        postEvent(EVENT_MODE_SHORT);
                        break;

                    case POWER_BUTTON:
                        postEvent(EVENT_POWER_SHORT);
                        break;
                }

            }
        }

        // Long press
        if (button.stableState == LOW &&
            !button.longReported &&
            (millis() - button.pressedTime >= longPressTime))
        {
            button.longReported = true;
            switch (pin)
            {
                case INSPECT_BUTTON:
                    postEvent(EVENT_INSPECT_LONG);
                    break;

                case MODE_BUTTON:
                    postEvent(EVENT_MODE_LONG);
                    break;

                case POWER_BUTTON:
                    postEvent(EVENT_POWER_LONG);
                    break;
            }
        }
    }

    button.lastReading = reading;
}
}

void updateButtons()
{
    processButton(
        INSPECT_BUTTON,
        inspectButton);

    processButton(
        MODE_BUTTON,
        modeButton);

    processButton(
        POWER_BUTTON,
        powerButton);
}
