#ifndef SENSORS_H
#define SENSORS_H

// Function prototypes for sensor-related functions
void initSensors();

float getTemperature();

float getHumidity();

float getPressure();

float getVOC();

const char* gasLevel (float voc);

#endif // SENSORS_H