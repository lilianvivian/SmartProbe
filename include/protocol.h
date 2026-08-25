#ifndef PROTOCOL_H
#define PROTOCOL_H

// =============================================================================
// SmartProbe LoRa wire protocol — shared by BOTH boards.
//
// The ground station (src/ground_station) and the remote node (src/node) each
// compile against this one file, so the vocabulary cannot drift. Previously the
// node accepted GUARDIAN|OVERRIDE and MONITOR|INSPECT as synonyms precisely
// because the two ends had never agreed on one spelling.
//
// The dashboard is JavaScript and cannot include this header, so it tolerates
// both the new and the legacy field names — see normalize() in
// dashboard/index.html. Keep the two in step when changing anything here.
// =============================================================================

#define PROTO_VERSION 2

// --- Radio parameters -------------------------------------------------------
// Must be identical on both ends or packets are simply never heard.
#define LORA_FREQ_HZ          868E6
#define LORA_SPREADING_FACTOR 7
#define LORA_BANDWIDTH_HZ     125E3
#define LORA_CODING_RATE      5

// --- Command message: ground station -> node --------------------------------
//   {"cmd":"SET_MODE","val":"MANUAL","seq":7}
//
// Naming reflects what the node actually does, which its old vocabulary did
// not. In the node's state machine (vTaskSystemEngine):
//
//   MANUAL  - forces buzzer, pump and fan to full output regardless of
//             conditions. Was "GUARDIAN"/"OVERRIDE".
//   AUTO    - resting state in which automatic breach detection drives the
//             actuators and sends SMS. Was "MONITOR"/"INSPECT".
//   SLEEP   - begin the deep-sleep cycle early. Was "TOGGLE"/"OFF".
//
// The dashboard still labels these "Guardian" and "Monitor" for the operator;
// only the wire format changed.
#define K_CMD        "cmd"
#define K_VAL        "val"
#define K_SEQ        "seq"

#define CMD_SET_MODE "SET_MODE"
#define CMD_POWER    "POWER"

#define VAL_MANUAL   "MANUAL"
#define VAL_AUTO     "AUTO"
#define VAL_SLEEP    "SLEEP"

// --- Telemetry message: node -> ground station ------------------------------
//   {"temp":22.6,"hum":57,"voc":841,"manual":false,"breach":true,"ack":7}
//
// State is reported as two independent booleans rather than one overloaded
// "mode" string. The old encoding collapsed them — reporting "GUARDIAN" when
// breached and "OVERRIDE" when manual — which forced both the ground station
// and the dashboard to infer one fact from the absence of another.
#define K_TEMP       "temp"
#define K_HUM        "hum"
#define K_VOC        "voc"
#define K_MANUAL     "manual"   // manual override engaged
#define K_BREACH     "breach"   // threshold or ML breach detected
#define K_ACK        "ack"      // seq of the last command the node applied

// --- Command acknowledgement ------------------------------------------------
// Every command carries a sequence number; the node echoes the last one it
// applied as "ack". Confirmation therefore becomes a fact rather than an
// inference from a state change, and a retry is safe because commands are
// idempotent — the same seq applied twice lands in the same state.
//
// SEQ_NONE marks "no command applied yet since boot".
#define SEQ_NONE 0
#define SEQ_MIN  1
#define SEQ_MAX  255

// Sequence numbers wrap 1..255, skipping SEQ_NONE so a fresh node is always
// distinguishable from one that has applied command 0.
static inline uint8_t protoNextSeq(uint8_t current) {
  return (current >= SEQ_MAX) ? SEQ_MIN : (uint8_t)(current + 1);
}

#endif // PROTOCOL_H
