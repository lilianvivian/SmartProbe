#ifndef GSM_H
#define GSM_H

#include <Arduino.h>

void initGSM();

bool sendSMS(String phoneNumber, String message);

bool checkModule();

void processGSM();

String sendCommand(String command, uint32_t timeout= 2000);

#endif