#ifndef LEDS_H
#define LEDS_H

void initLEDs();

// Individual LED controls
void greenLed(bool state);
void yellowLed(bool state);
void redLed(bool state);

// Update all LEDs at once
void updateLEDs(
    bool green,
    bool yellow,
    bool red
);

#endif