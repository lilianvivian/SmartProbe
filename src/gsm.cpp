#include "gsm.h"


HardwareSerial sim800(1);

// Current pin allocation
#define GSM_TX_PIN 1  // ESP32 TX -> SIM RX
#define GSM_RX_PIN 2  // ESP32 RX <- SIM TX

//--------------------------------------------------
// Send Command
//--------------------------------------------------

String sendCommand(String command, uint32_t timeout)
{
    while (sim800.available())
    {
        sim800.read();
    }

    Serial.print(">> ");
    Serial.println(command);

    sim800.println(command);

    String response = "";

    unsigned long start = millis();

    while (millis() - start < timeout)
    {
        while (sim800.available())
        {
            response += (char)sim800.read();
        }

        if (response.indexOf("OK") >= 0 ||
            response.indexOf("ERROR") >= 0)
        {
            break;
        }

        delay(5);
    }

    Serial.print("<< ");
    Serial.println(response);

    return response;
}

//--------------------------------------------------
// Initialize GSM
//--------------------------------------------------

void initGSM()
{
    Serial.println("Initializing SIM800L...");

    sim800.begin(115200, SERIAL_8N1, GSM_RX_PIN, GSM_TX_PIN);

    delay(3000);

    while (sim800.available())
        sim800.read();

    sim800.println("AT");

    delay(1000);

    while (sim800.available())
    {
        Serial.write(sim800.read());
    }

    Serial.println("Finished GSM test.");
}
//--------------------------------------------------
// Check Module
//--------------------------------------------------

bool checkModule()
{
    String response = sendCommand("AT");

    return response.indexOf("OK") != -1;
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