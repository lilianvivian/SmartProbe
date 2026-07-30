#ifndef GSM_H
#define GSM_H

#include <Arduino.h>

void initGSM();
void updateGSM();

bool gsmReady();
bool gsmRegistered();

bool sendSMS(const String &number, const String &message);

int getSignalStrength();
String getOperator();

#endif