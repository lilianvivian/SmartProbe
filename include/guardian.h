#ifndef GUARDIAN_H
#define GUARDIAN_H

enum GrainStatus
{
    SAFE,
    WARNING,
    CRITICAL
};

enum GrainCause
{
    NONE,
    TEMPERATURE,
    HUMIDITY,
    VOC
};

struct GuardianDecision
{
    // Overall storage condition
    GrainStatus status;

    // Main factor contributing to the condition
    GrainCause cause;

    // Recommended immediate actions
    bool runFan;
    bool soundAlarm;

    bool greenLed;
    bool yellowLed;
    bool redLed;

    bool sendSMS;

    const char* recommendation;
};

GuardianDecision evaluateStorage(
    float temperature,
    float humidity,
    float voc
);

const char* statusToString(GrainStatus status);
const char* causeToString(GrainCause cause);

#endif