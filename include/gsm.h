#ifndef GSM_H
#define GSM_H

#include <Arduino.h>

void initGSM();

bool sendSMS(String phoneNumber, String message);

bool checkModule();

void processGSM();

#endif