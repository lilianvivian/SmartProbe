#include "gsm.h"

HardwareSerial sim800(1);

// Current pin allocation
#define GSM_TX_PIN 9     // ESP32 TX -> SIM RX
#define GSM_RX_PIN 10    // ESP32 RX <- SIM TX

//--------------------------------------------------
// Send Command
//--------------------------------------------------

String sendCommand(String command, uint32_t timeout = 2000)
{
    while (sim800.available())
        sim800.read();

    sim800.println(command);

    uint32_t start = millis();
    String response = "";

    while (millis() - start < timeout)
    {
        while (sim800.available())
        {
            response += (char)sim800.read();
        }
    }

    return response;
}

//--------------------------------------------------
// Initialize GSM
//--------------------------------------------------

void initGSM()
{
    sim800.begin(9600, SERIAL_8N1, GSM_RX_PIN, GSM_TX_PIN);

    delay(2000);

    sendCommand("AT");

    sendCommand("ATE0");

    sendCommand("AT+CMGF=1");
}

//--------------------------------------------------
// Check Module
//--------------------------------------------------

bool checkModule()
{
    String response = sendCommand("AT");

    return response.indexOf("OK") >= 0;
}

//--------------------------------------------------
// Send SMS
//--------------------------------------------------

bool sendSMS(String phoneNumber, String message)
{
    sim800.print("AT+CMGS=\"");
    sim800.print(phoneNumber);
    sim800.println("\"");

    delay(1000);

    sim800.print(message);

    delay(500);

    sim800.write(26);

    delay(5000);

    String response = "";

    while (sim800.available())
    {
        response += (char)sim800.read();
    }

    return response.indexOf("OK") >= 0;
}

//--------------------------------------------------

void processGSM()
{

}