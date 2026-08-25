#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// --- Pin Definitions ---
#define LORA_CS     10
#define LORA_RST    9
#define LORA_DIO0   2

#define BTN_POWER    3
#define BTN_GUARDIAN 4
#define BTN_MONITOR  5

#define LORA_FREQ    868E6
#define DEBOUNCE_MS  250

// --- Cooperative schedule (ms) ---
// Every job here is a short periodic poll: nothing blocks, nothing waits on
// I/O, and no job needs to preempt another. That is a super-loop, so this
// runs on one stack instead of paying ~900 bytes of RTOS task stacks, TCBs,
// idle task, queue and mutex out of a 2 KB part.
#define INPUT_INTERVAL_MS    35
#define RADIO_INTERVAL_MS    40
#define DISPLAY_INTERVAL_MS  400

// Command codes. The JSON wire format is rebuilt from flash at transmit time,
// so the queue holds one byte per command rather than two 10-char strings.
enum CommandCode : uint8_t {
  CMD_GUARDIAN,
  CMD_MONITOR,
  CMD_SLEEP
};

struct BaseStationState {
  float temp = 0.0;
  float hum = 0.0;
  int voc = 0;
  char mode[10] = "MONITOR";
  float emaTemp = 0.0;
  float emaHum = 0.0;
  bool lastDataValid = false;
  unsigned long lastRxTime = 0;
  unsigned int packets = 0;   // received since power-up, reported to the dashboard
};

LiquidCrystal_I2C lcd(0x27, 16, 2);
BaseStationState baseState;

// --- Command ring buffer ---------------------------------------------------
// Replaces the FreeRTOS queue: 6 bytes of .bss against roughly 85 for a queue
// object. Single-threaded, so no locking is needed anywhere in this sketch.
#define CMD_QUEUE_LEN 4
static uint8_t cmdQ[CMD_QUEUE_LEN];
static uint8_t cmdHead = 0;
static uint8_t cmdTail = 0;

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
// parser and no JSON library — the whole USB input path costs zero persistent
// SRAM. G = arm, M = disarm, P = sleep.
static void handleSerialCommands() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    switch (c) {
      case 'G': queueCommand(CMD_GUARDIAN); break;
      case 'M': queueCommand(CMD_MONITOR);  break;
      case 'P': queueCommand(CMD_SLEEP);    break;
      default:  continue;   // newlines and stray bytes are ignored silently
    }
    // Echoed so the USB half of the chain is observable. No '{' in this line,
    // so the dashboard's parser skips it.
    Serial.print(F("[CMD] "));
    Serial.println(c);
  }
}

// Measured headroom beats assumed headroom: the gap between the top of the
// heap and the current stack pointer.
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
  if (!LoRa.begin(LORA_FREQ)) {
    lcd.setCursor(0, 1);
    lcd.print(F("LoRa FAIL!"));
    while (1);
  }

  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.enableCrc();
  // Deliberately NO LoRa.receive() here. receive() selects RX_CONTINUOUS, but
  // parsePacket() below is written for RX_SINGLE: when it finds the radio in
  // any other mode it writes REG_FIFO_ADDR_PTR back to 0 before switching. A
  // packet then gets read from the start of the FIFO instead of its own offset,
  // which decodes as binary garbage. Polling parsePacket() alone lets the
  // library own the mode consistently.

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
    queueCommand(CMD_GUARDIAN);
  }

  if (digitalRead(BTN_MONITOR) == LOW && (now - lastMonitor > DEBOUNCE_MS)) {
    lastMonitor = now;
    queueCommand(CMD_MONITOR);
  }

  // The dashboard is simply a fourth input onto the same queue.
  handleSerialCommands();
}

// --- Job 2: radio ----------------------------------------------------------
static void serviceRadio() {
  static char rxBuffer[96];
  uint8_t txCode;

  // 1. Send one queued command per pass
  if (dequeueCommand(&txCode)) {
    LoRa.idle();
    LoRa.beginPacket();
    // Wire format rebuilt from flash — F() keeps these out of SRAM entirely.
    switch (txCode) {
      case CMD_GUARDIAN: LoRa.print(F("{\"cmd\":\"SET_MODE\",\"val\":\"GUARDIAN\"}")); break;
      case CMD_MONITOR:  LoRa.print(F("{\"cmd\":\"SET_MODE\",\"val\":\"MONITOR\"}"));  break;
      case CMD_SLEEP:    LoRa.print(F("{\"cmd\":\"POWER\",\"val\":\"OFF\"}"));         break;
    }
    // endPacket() returns 0 if the radio never completed the transmission.
    // Reporting it separates "the ground station never sent" from "it sent and
    // the node did not hear it" - otherwise both look identical.
    bool sent = LoRa.endPacket();

    Serial.print(F("[TX] "));
    Serial.print(txCode == CMD_GUARDIAN ? 'G' : txCode == CMD_MONITOR ? 'M' : 'P');
    Serial.println(sent ? F(" ok") : F(" FAILED"));
    return;                 // let parsePacket() re-arm RX on the next pass
  }

  // 2. Read any incoming packet into the fixed buffer
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
  // fields this board never parses (voc, breach) and needs no output buffer.
  if (bytesRead > 1 && rxBuffer[bytesRead - 1] == '}') {
    Serial.write((const uint8_t *)rxBuffer, bytesRead - 1);
    Serial.print(F(",\"rssi\":"));    Serial.print(LoRa.packetRssi());
    Serial.print(F(",\"snr\":"));     Serial.print(LoRa.packetSnr(), 1);
    Serial.print(F(",\"packets\":")); Serial.print(baseState.packets);
    Serial.println('}');
  } else {
    Serial.print(F("[RX MALFORMED] "));
    Serial.println(rxBuffer);
  }

  // Field extraction by string search — cheaper than a JSON parser here.
  char *tempPtr = strstr(rxBuffer, "\"temp\":");
  char *humPtr  = strstr(rxBuffer, "\"hum\":");
  char *modePtr = strstr(rxBuffer, "\"mode\":");
  if (tempPtr == NULL) return;

  baseState.temp = atof(tempPtr + 7);
  if (humPtr != NULL) baseState.hum = atof(humPtr + 6);

  if (modePtr != NULL) {
    char *modeStart = strchr(modePtr + 7, '"');
    if (modeStart != NULL) {
      modeStart++;
      char *modeEnd = strchr(modeStart, '"');
      if (modeEnd != NULL) {
        size_t len = (size_t)(modeEnd - modeStart);
        if (len < sizeof(baseState.mode)) {
          strncpy(baseState.mode, modeStart, len);
          baseState.mode[len] = '\0';
        }
      }
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

  if (baseState.mode[0] == 'G' || baseState.mode[0] == 'O') {
    lcd.print(F("GARD "));
  } else {
    lcd.print(F("INSP "));
  }

  lcd.print(F("T:"));
  lcd.print((int)baseState.emaTemp);
  lcd.print(F("C H:"));
  lcd.print((int)baseState.emaHum);
  lcd.print(F("%   "));

  lcd.setCursor(0, 1);
  lcd.print(F("Status: Active  "));
}

void loop() {
  static unsigned long tInputs = 0, tRadio = 0, tDisplay = 0;
  unsigned long now = millis();

  if (now - tInputs >= INPUT_INTERVAL_MS)    { tInputs  = now; pollInputs();   }
  if (now - tRadio >= RADIO_INTERVAL_MS)     { tRadio   = now; serviceRadio(); }
  if (now - tDisplay >= DISPLAY_INTERVAL_MS) { tDisplay = now; updateDisplay(); }
}
