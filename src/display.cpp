#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "display.h"

//--------------------------------------------------
// LCD Configuration
//--------------------------------------------------
#define LCD_ADDRESS 0x27
#define LCD_COLUMNS 16
#define LCD_ROWS    2

LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLUMNS, LCD_ROWS);

//--------------------------------------------------
// Dashboard Variables
//--------------------------------------------------
unsigned long displayPreviousMillis = 0;
byte currentScreen = 0;

//--------------------------------------------------
// Initialize LCD
//--------------------------------------------------
void initDisplay()
{
    Wire.begin(21, 20);

    lcd.init();
    lcd.backlight();
    lcd.clear();
}

//--------------------------------------------------
// Boot Screen
//--------------------------------------------------
void showBootScreen()
{
    lcd.clear();

    lcd.setCursor(2,0);
    lcd.print("SMART PROBE");

    lcd.setCursor(2,1);
    lcd.print("Firmware 1.0");

    delay(2000);
}

//--------------------------------------------------
// Initializing Screen
//--------------------------------------------------
void showInitializing()
{
    lcd.clear();

    lcd.setCursor(0,0);
    lcd.print("Initializing...");
}

//--------------------------------------------------
// Component Progress
//--------------------------------------------------
void updateProgress(const char* item)
{
    lcd.clear();

    lcd.setCursor(0,0);
    lcd.print("Checking...");

    lcd.setCursor(0,1);
    lcd.print(item);

    delay(800);
}

//--------------------------------------------------
// Ready Screen
//--------------------------------------------------
void showReady()
{
    lcd.clear();

    lcd.setCursor(2,0);
    lcd.print("SYSTEM READY");

    lcd.setCursor(3,1);
    lcd.print("Probe SP001");

    delay(1500);
}

//--------------------------------------------------
// Dashboard
//--------------------------------------------------
void showDashboard(
    float temperature,
    float humidity,
    float voc,
    const char* gas,
    const char* status,
    const char* trend,
    int nextInspectionDays)
{
    if (millis() - displayPreviousMillis >= 3000)
    {
        displayPreviousMillis = millis();

        currentScreen++;

        if (currentScreen > 2)
            currentScreen = 0;
    }

    lcd.clear();

    switch(currentScreen)
    {
        //--------------------------------------------------
        // Screen 1 : Live Readings
        //--------------------------------------------------
        case 0:

            lcd.setCursor(0,0);
            lcd.print("T:");
            lcd.print(temperature,1);
            lcd.print(" H:");
            lcd.print(humidity,0);
            lcd.print("%");

            lcd.setCursor(0,1);
            lcd.print("Status:");
            lcd.print(status);

            break;

        //--------------------------------------------------
        // Screen 2 : VOC + AI Trend
        //--------------------------------------------------
        case 1:

            lcd.setCursor(0,0);
            lcd.print("Gas:");
            lcd.print(gas);

            lcd.setCursor(0,1);
            lcd.print("Trend:");
            lcd.print(trend);

            break;

        //--------------------------------------------------
        // Screen 3 : Next Inspection
        //--------------------------------------------------
        case 2:

            lcd.setCursor(0,0);
            lcd.print("Inspect:");
            lcd.print(nextInspectionDays);
            lcd.print("d");

            lcd.setCursor(0,1);
            lcd.print("ID:SP001");

            break;
    }
}