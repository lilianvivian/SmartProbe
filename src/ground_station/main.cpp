#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "protocol.h"
#include "pins.h"

#define DEBOUNCE_MS  250

// --- Cooperative schedule (ms) ---
// Every job here is a short periodic poll: nothing blocks, nothing waits on
// I/O, and no job needs to preempt another. That is a super-loop, so this runs
// on one stack instead of paying ~900 bytes of RTOS task stacks, TCBs, idle
// task, queue and mutex out of a 2 KB part.
#define INPUT_INTERVAL_MS    35
#define RADIO_INTERVAL_MS    40
#define DISPLAY_INTERVAL_MS  400

// A command is retransmitted until the node echoes its seq back as "ack".
// This is what makes the link self-healing: LoRa is half-duplex, so a command
// sent while the node happens to be transmitting telemetry is simply lost.
#define ACK_TIMEOUT_MS  1200
#define MAX_RETRIES     3

// Local command codes. The wire spelling lives in protocol.h.
enum CommandCode : uint8_t {
  CMD_MANUAL,
  CMD_AUTO,
  CMD_SLEEP
};

struct BaseStationState {
  float temp = 0.0;
  float hum = 0.0;
  int voc = 0;
  float emaTemp = 0.0;
  float emaHum = 0.0;
  bool manual = false;          // node is in manual override
  bool breach = false;          // node reports a threshold/ML breach
  uint8_t ack = SEQ_NONE;       // last command seq the node applied
  bool lastDataValid = false;
  unsigned long lastRxTime = 0;
  unsigned int packets = 0;
};

LiquidCrystal_I2C lcd(0x27, 16, 2);
BaseStationState baseState;

// --- Command ring buffer ---------------------------------------------------
// 6 bytes of .bss against roughly 85 for a FreeRTOS queue object.
// Single-threaded, so nothing needs locking anywhere in this sketch.
#define CMD_QUEUE_LEN 4
static uint8_t cmdQ[CMD_QUEUE_LEN];
static uint8_t cmdHead = 0;
static uint8_t cmdTail = 0;

// --- In-flight command tracking --------------------------------------------
static uint8_t inFlightCmd  = 0;
static uint8_t inFlightSeq  = SEQ_NONE;   // SEQ_NONE => nothing awaiting ack
static uint8_t retriesLeft  = 0;
static unsigned long nextRetryAt = 0;
static uint8_t lastSeqUsed  = SEQ_NONE;

// Single entry point for both input sources, so a button press and a dashboard
// click are indistinguishable downstream.
static void queueCommand(uint8_t code) {
  uint8_t next = (uint8_t)((cmdHead + 1) % CMD_QUEUE_LEN);
  if (next == cmdTail) return;          // full: drop rather than overwrite
  cmdQ[cmdHead] = code;
  cmdHead = next;
}

static bool dequeueCommand(uint8_t *out) {
  if (cmdHead == cmdTail) return false;
  *out = cmdQ[cmdTail];
  cmdTail = (uint8_t)((cmdTail + 1) % CMD_QUEUE_LEN);
  return true;
}

// --- USB command input -----------------------------------------------------
// The dashboard sends one byte per command, so this needs no line buffer, no
// parser and no JSON library. G = manual override, M = auto, P = sleep.
static void handleSerialCommands() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    switch (c) {
      case 'G': queueCommand(CMD_MANUAL); break;
      case 'M': queueCommand(CMD_AUTO);   break;
      case 'P': queueCommand(CMD_SLEEP);  break;
      default:  continue;   // newlines and stray bytes are ignored silently
    }
    // Echoed so the USB half of the chain is observable. No '{' in this line,
    // so the dashboard's telemetry parser skips it.
    Serial.print(F("[CMD] "));
    Serial.println(c);
  }
}

// Measured headroom beats assumed headroom.
extern unsigned int __heap_start;
extern void *__brkval;

static int freeRam() {
  int marker;
  return (int)&marker - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}

void setup() {
  Serial.begin(115200);

  pinMode(BTN_POWER, INPUT_PULLUP);
  pinMode(BTN_GUARDIAN, INPUT_PULLUP);
  pinMode(BTN_MONITOR, INPUT_PULLUP);

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Ground Station"));

  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ_HZ)) {
    lcd.setCursor(0, 1);
    lcd.print(F("LoRa FAIL!"));
    while (1);
  }

  LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
  LoRa.setSignalBandwidth(LORA_BANDWIDTH_HZ);
  LoRa.setCodingRate4(LORA_CODING_RATE);
  LoRa.enableCrc();
  // Deliberately NO LoRa.receive() here. receive() selects RX_CONTINUOUS, but
  // parsePacket() is written for RX_SINGLE: when it finds the radio in any
  // other mode it writes REG_FIFO_ADDR_PTR back to 0 before switching, so a
  // packet gets read from the start of the FIFO instead of its own offset and
  // decodes as binary garbage.

  Serial.print(F("Free SRAM: "));
  Serial.println(freeRam());
}

// --- Job 1: buttons and USB ------------------------------------------------
static void pollInputs() {
  unsigned long now = millis();
  static unsigned long lastPower = 0, lastGuardian = 0, lastMonitor = 0;

  if (digitalRead(BTN_POWER) == LOW && (now - lastPower > DEBOUNCE_MS)) {
    lastPower = now;
    queueCommand(CMD_SLEEP);
  }

  if (digitalRead(BTN_GUARDIAN) == LOW && (now - lastGuardian > DEBOUNCE_MS)) {
    lastGuardian = now;
    queueCommand(CMD_MANUAL);
  }

  if (digitalRead(BTN_MONITOR) == LOW && (now - lastMonitor > DEBOUNCE_MS)) {
    lastMonitor = now;
    queueCommand(CMD_AUTO);
  }

  // The dashboard is simply a fourth input onto the same queue.
  handleSerialCommands();
}

// --- Radio: transmit -------------------------------------------------------
static void transmitCommand(uint8_t code, uint8_t seq) {
  LoRa.idle();
  LoRa.beginPacket();
  // Built from flash — F() keeps the wire strings out of SRAM entirely.
  LoRa.print(F("{\"" K_CMD "\":\""));
  LoRa.print(code == CMD_SLEEP ? F(CMD_POWER) : F(CMD_SET_MODE));
  LoRa.print(F("\",\"" K_VAL "\":\""));
  switch (code) {
    case CMD_MANUAL: LoRa.print(F(VAL_MANUAL)); break;
    case CMD_AUTO:   LoRa.print(F(VAL_AUTO));   break;
    case CMD_SLEEP:  LoRa.print(F(VAL_SLEEP));  break;
  }
  LoRa.print(F("\",\"" K_SEQ "\":"));
  LoRa.print(seq);
  LoRa.print(F("}"));

  // endPacket() returns 0 if the radio never completed the transmission, which
  // separates "never sent" from "sent but not heard".
  bool sent = LoRa.endPacket();

  Serial.print(F("[TX] "));
  Serial.print(code == CMD_MANUAL ? 'G' : code == CMD_AUTO ? 'M' : 'P');
  Serial.print(F(" seq="));
  Serial.print(seq);
  Serial.println(sent ? F(" ok") : F(" FAILED"));
}

// --- Job 2: radio ----------------------------------------------------------
static void serviceRadio() {
  static char rxBuffer[112];
  unsigned long now = millis();

  // 1. Retransmit an unacknowledged command, or start the next queued one.
  if (inFlightSeq != SEQ_NONE) {
    if ((long)(now - nextRetryAt) >= 0) {
      if (retriesLeft > 0) {
        retriesLeft--;
        nextRetryAt = now + ACK_TIMEOUT_MS;
        Serial.print(F("[RETRY] seq="));
        Serial.println(inFlightSeq);
        transmitCommand(inFlightCmd, inFlightSeq);
        return;
      }
      Serial.print(F("[GIVEUP] seq="));
      Serial.println(inFlightSeq);
      inFlightSeq = SEQ_NONE;
    }
  } else {
    uint8_t code;
    if (dequeueCommand(&code)) {
      lastSeqUsed = protoNextSeq(lastSeqUsed);
      inFlightCmd = code;
      inFlightSeq = lastSeqUsed;
      retriesLeft = MAX_RETRIES;
      nextRetryAt = now + ACK_TIMEOUT_MS;
      transmitCommand(code, inFlightSeq);
      return;         // let parsePacket() re-arm RX on the next pass
    }
  }

  // 2. Read any incoming packet into the fixed buffer.
  int packetSize = LoRa.parsePacket();
  if (packetSize <= 0) return;

  int bytesRead = 0;
  while (LoRa.available() && bytesRead < (int)(sizeof(rxBuffer) - 1)) {
    rxBuffer[bytesRead++] = (char)LoRa.read();
  }
  rxBuffer[bytesRead] = '\0';

  baseState.packets++;

  // One line for the dashboard: the node's own payload with radio stats spliced
  // in before the closing brace. Splicing rather than rebuilding preserves
  // every field verbatim and needs no output buffer of its own.
  if (bytesRead > 1 && rxBuffer[bytesRead - 1] == '}') {
    Serial.write((const uint8_t *)rxBuffer, bytesRead - 1);
    Serial.print(F(",\"rssi\":"));    Serial.print(LoRa.packetRssi());
    Serial.print(F(",\"snr\":"));     Serial.print(LoRa.packetSnr(), 1);
    Serial.print(F(",\"packets\":")); Serial.print(baseState.packets);
    Serial.println('}');
  } else {
    Serial.print(F("[RX MALFORMED] "));
    Serial.println(rxBuffer);
    return;
  }

  // Field extraction by string search — cheaper than a JSON parser here.
  char *p;

  if ((p = strstr(rxBuffer, "\"" K_TEMP "\":")) != NULL)
    baseState.temp = atof(p + sizeof(K_TEMP) + 2);
  if ((p = strstr(rxBuffer, "\"" K_HUM "\":")) != NULL)
    baseState.hum = atof(p + sizeof(K_HUM) + 2);
  if ((p = strstr(rxBuffer, "\"" K_VOC "\":")) != NULL)
    baseState.voc = atoi(p + sizeof(K_VOC) + 2);

  // Two explicit booleans instead of one overloaded mode string: no inference.
  if ((p = strstr(rxBuffer, "\"" K_MANUAL "\":")) != NULL)
    baseState.manual = (strncmp(p + sizeof(K_MANUAL) + 2, "true", 4) == 0);
  if ((p = strstr(rxBuffer, "\"" K_BREACH "\":")) != NULL)
    baseState.breach = (strncmp(p + sizeof(K_BREACH) + 2, "true", 4) == 0);

  // Acknowledgement closes the loop on the in-flight command.
  if ((p = strstr(rxBuffer, "\"" K_ACK "\":")) != NULL) {
    baseState.ack = (uint8_t)atoi(p + sizeof(K_ACK) + 2);
    if (inFlightSeq != SEQ_NONE && baseState.ack == inFlightSeq) {
      Serial.print(F("[ACK] seq="));
      Serial.println(inFlightSeq);
      inFlightSeq = SEQ_NONE;
    }
  }

  if (!baseState.lastDataValid) {
    baseState.emaTemp = baseState.temp;
    baseState.emaHum  = baseState.hum;
  } else {
    baseState.emaTemp = (0.3f * baseState.temp) + (0.7f * baseState.emaTemp);
    baseState.emaHum  = (0.3f * baseState.hum)  + (0.7f * baseState.emaHum);
  }

  baseState.lastDataValid = true;
  baseState.lastRxTime = millis();
}

// --- Job 3: display --------------------------------------------------------
static void updateDisplay() {
  bool offline = (millis() - baseState.lastRxTime > 40000) || !baseState.lastDataValid;

  lcd.setCursor(0, 0);
  if (offline) {
    lcd.print(F("LINK: SLEEP/OFF "));
    lcd.setCursor(0, 1);
    lcd.print(F("Waiting Node... "));
    return;
  }

  // The two booleans make this direct: manual wins, then breach, then resting.
  if (baseState.manual)      lcd.print(F("MANU "));
  else if (baseState.breach) lcd.print(F("ALRM "));
  else                       lcd.print(F("AUTO "));

  lcd.print(F("T:"));
  lcd.print((int)baseState.emaTemp);
  lcd.print(F("C H:"));
  lcd.print((int)baseState.emaHum);
  lcd.print(F("%   "));

  lcd.setCursor(0, 1);
  if (inFlightSeq != SEQ_NONE) {
    lcd.print(F("Sending cmd...  "));
  } else {
    lcd.print(F("VOC:"));
    lcd.print(baseState.voc);
    lcd.print(F("        "));
  }
}

void loop() {
  static unsigned long tInputs = 0, tRadio = 0, tDisplay = 0;
  unsigned long now = millis();

  if (now - tInputs >= INPUT_INTERVAL_MS)    { tInputs  = now; pollInputs();   }
  if (now - tRadio >= RADIO_INTERVAL_MS)     { tRadio   = now; serviceRadio(); }
  if (now - tDisplay >= DISPLAY_INTERVAL_MS) { tDisplay = now; updateDisplay(); }
}
