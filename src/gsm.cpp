#include "gsm.h"
#include "pins.h"

HardwareSerial sim800(2);

//--------------------------------------------------

static bool moduleReady = false;

static const unsigned long REGISTRATION_TIMEOUT = 30000;
static const int AT_ATTEMPTS = 5;

//--------------------------------------------------

static void clearBuffer()
{
    while (sim800.available())
        sim800.read();
}

//--------------------------------------------------

static String readResponse(unsigned long timeout)
{
    String response = "";

    unsigned long start = millis();

    while (millis() - start < timeout)
    {
        while (sim800.available())
        {
            response += (char)sim800.read();
        }
    }

    if(response.length())
        Serial.println(response);

    return response;
}

//--------------------------------------------------

static String sendCommand(String cmd, unsigned long timeout)
{
    clearBuffer();

    Serial.print(">> ");
    Serial.println(cmd);

    sim800.println(cmd);

    return readResponse(timeout);
}

//--------------------------------------------------

// The SIM800 has its own supply and does not reset when the ESP32 does, so it
// can still be sitting at the SMS ">" prompt from a previous run. ESC + Ctrl-Z
// aborts that prompt; without it the modem swallows "AT" as message text and
// never answers OK.
static void abortPendingPrompt()
{
    sim800.write(27);

    delay(500);

    sim800.write(26);

    delay(1000);
}

void initGSM()
{
    Serial.println();
    Serial.println("================================");
    Serial.println(" SMART PROBE GSM INITIALIZATION");
    Serial.println("================================");

    sim800.begin(9600, SERIAL_8N1, GSM_RX_PIN, GSM_TX_PIN);

    delay(8000);

    abortPendingPrompt();

    moduleReady = false;

    for(int attempt = 1; attempt <= AT_ATTEMPTS; attempt++)
    {
        Serial.print("AT attempt ");
        Serial.print(attempt);
        Serial.print(" of ");
        Serial.println(AT_ATTEMPTS);

        if(sendCommand("AT",1500).indexOf("OK") != -1)
        {
            moduleReady = true;

            break;
        }

        delay(1000);
    }

    if(!moduleReady)
    {
        Serial.println("GSM NOT DETECTED!");

        return;
    }

    sendCommand("AT+CMEE=2",1500);

    sendCommand("AT+CSCS=\"GSM\"",1500);

    sendCommand("AT+CMGF=1",1500);

    sendCommand("AT+CPIN?",1500);

    sendCommand("AT+CSQ",1500);

    sendCommand("AT+CREG?",1500);

    sendCommand("AT+COPS?",3000);

    Serial.println("GSM READY");
}

bool gsmReady()
{
    return moduleReady;
}

bool gsmRegistered()
{
    if(!moduleReady)
        return false;

    String response = sendCommand("AT+CREG?",2000);

    return response.indexOf("+CREG: 0,1")!=-1 ||
           response.indexOf("+CREG: 0,5")!=-1;
}

//--------------------------------------------------

// A SIM800 normally needs 10-30s to attach after power-up, so a single query
// is not enough to conclude the network is unavailable.
static bool waitForRegistration(unsigned long timeout)
{
    unsigned long start = millis();

    while(millis() - start < timeout)
    {
        if(gsmRegistered())
            return true;

        Serial.println("Waiting for network registration...");

        delay(2000);
    }

    return false;
}

int getSignalStrength()
{
    if(!moduleReady)
        return -1;

    String response = sendCommand("AT+CSQ",1500);

    int pos = response.indexOf("+CSQ:");

    if(pos==-1)
        return -1;

    int comma = response.indexOf(",",pos);

    String value = response.substring(pos+6,comma);

    value.trim();

    return value.toInt();
}

String getOperator()
{
    if(!moduleReady)
        return "";

    String response = sendCommand("AT+COPS?",3000);

    return response;
}

bool sendSMS(const String &number,const String &message)
{
    if(!moduleReady)
    {
        Serial.println("GSM offline.");

        return false;
    }

    if(!waitForRegistration(REGISTRATION_TIMEOUT))
    {
        Serial.println("Network unavailable.");

        return false;
    }

    clearBuffer();

    sim800.print("AT+CMGS=\"");
    sim800.print(number);
    sim800.println("\"");

    String response = readResponse(3000);

    if(response.indexOf(">")==-1)
    {
        Serial.println("CMGS failed.");

        abortPendingPrompt();

        return false;
    }

    sim800.print(message);

    sim800.write(26);

    response = readResponse(10000);

    if(response.indexOf("+CMGS")!=-1)
    {
        Serial.println("SMS SENT SUCCESSFULLY");

        return true;
    }

    Serial.println("SMS FAILED");

    abortPendingPrompt();

    return false;
}

void updateGSM()
{
    while(sim800.available())
    {
        Serial.write(sim800.read());
    }
}