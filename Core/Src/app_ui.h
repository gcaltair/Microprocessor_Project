#ifndef APP_UI_H
#define APP_UI_H

#include <stdbool.h>
#include <stdint.h>
#include "main.h"

typedef enum {
    APP_PAGE_MAIN = 0,
    APP_PAGE_POSE,
    APP_PAGE_DEBUG,
    APP_PAGE_COUNT
} AppDisplayPage;

typedef enum {
    APP_BUTTON_NONE = 0,
    APP_BUTTON_SHORT,
    APP_BUTTON_LONG
} AppButtonEvent;

void App_Init(void);
void App_Task10ms(float dt);
void App_BackgroundTask(void);
bool App_MotorsAllowed(void);
void App_ApplySpeedLimit(void);
void App_RequestEmergencyStop(const char *reason);
void App_ClearEmergencyStop(void);
void App_StartReturnHome(void);
void App_SetTelemetryEnabled(bool enabled);
bool App_GetTelemetryEnabled(void);
uint16_t App_GetBatteryMillivolts(void);
uint16_t App_GetSpeedLimitCmps(void);
const char *App_GetSafetyReason(void);

#endif
