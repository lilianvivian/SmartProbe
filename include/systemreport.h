#ifndef SYSTEMREPORT_H
#define SYSTEMREPORT_H

#include "guardian.h"
#include "tinyml.h"

struct SystemReport
{
    float temperature;
    float humidity;
    float pressure;
    float voc;

    GuardianDecision guardian;
    Prediction prediction;
};

extern SystemReport report;

#endif