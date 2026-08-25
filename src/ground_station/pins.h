#ifndef GS_PINS_H
#define GS_PINS_H

// Ground station (Arduino Uno) pin map.
//
// Kept beside main.cpp rather than in include/ so it cannot collide with the
// node's own pins.h — the compiler searches the including file's directory
// first, so each board picks up its own.

// --- LoRa (SX127x over hardware SPI) ---
// SPI itself is fixed on the Uno: MOSI 11, MISO 12, SCK 13. Only the control
// lines are assignable.
#define LORA_CS      10
#define LORA_RST      9
#define LORA_DIO0     2

// --- Push buttons (active LOW, internal pull-ups) ---
// Labels follow the operator's vocabulary; on the wire these map to
// MANUAL / AUTO / SLEEP — see include/protocol.h.
#define BTN_POWER     3
#define BTN_GUARDIAN  4
#define BTN_MONITOR   5

// --- 16x2 I2C LCD ---
// Uses the Uno's fixed I2C pins (A4 = SDA, A5 = SCL); address set in main.cpp.

#endif // GS_PINS_H
