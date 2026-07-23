#ifndef TINYML_H
#define TINYML_H


const int HISTORY_SIZE = 10;
enum Trend
{
    STABLE,
    IMPROVING,
    SLOWLY_DETERIORATING,
    RAPIDLY_DETERIORATING
};

struct Prediction
{
    Trend trend;

    int nextInspectionDays;

    float confidence;
};

void initTinyML();

Prediction predictInspection(
    float temperature,
    float humidity,
    float voc
);

const char* trendToString(Trend trend);
const char* trendToLCDString(Trend trend);

#endif