#ifndef DISPLAY_H
#define DISPLAY_H

//--------------------------------------------------
// OLED Initialization
//--------------------------------------------------
void initDisplay();

//--------------------------------------------------
// Boot Screens
//--------------------------------------------------
void showBootScreen();
void showInitializing();
void updateProgress(const char* item);
void showReady();

//--------------------------------------------------
// Main Dashboard
//--------------------------------------------------
void showDashboard(
    float temperature,
    float humidity,
    float voc,
    const char* gas,
    const char* status,
    const char* trend,
    int nextInspectionHours
);

#endif // DISPLAY_H