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

typedef enum {
    APP_SAFETY_OK = 0,
    APP_SAFETY_STOP,
    APP_SAFETY_EMERGENCY,
    APP_SAFETY_LOW_BATTERY,
    APP_SAFETY_WATCHDOG,
    APP_SAFETY_MOTOR_STALL,
    APP_SAFETY_LIDAR_RECOVERY
} AppSafetyCode;

typedef enum {
    APP_BENCHMARK_IDLE = 0,
    APP_BENCHMARK_OUTBOUND,
    APP_BENCHMARK_RETURNING,
    APP_BENCHMARK_COMPLETE
} AppBenchmarkState;

typedef struct {
    uint16_t battery_mv;
    uint16_t speed_limit_cmps;
    uint8_t safety_code;
    uint8_t emergency_stop;
    uint8_t lidar_stream_active;
    uint8_t benchmark_state;
    uint32_t lidar_recovery_count;
    uint32_t control_age_ms;
    uint32_t mission_elapsed_ms;
    uint32_t exit_time_ms;
    uint32_t return_time_ms;
} AppStatusSnapshot_t;

void App_Init(void);
void App_RecordControlTick(void);
void App_ServiceTask(void);
bool App_MotorsAllowed(void);
void App_RequestEmergencyStop(const char *reason);
void App_RequestEmergencyStopFromIsr(void);
void App_ClearEmergencyStop(void);
void App_StartExploration(void);
void App_StartReturnHome(void);
void App_SetTelemetryEnabled(bool enabled);
bool App_GetTelemetryEnabled(void);
uint16_t App_GetBatteryMillivolts(void);
uint16_t App_GetSpeedLimitCmps(void);
const char *App_GetSafetyReason(void);
void App_BeginBenchmark(void);
void App_RecordExitReached(void);
void App_RecordReturnReached(void);
void App_GetStatusSnapshot(AppStatusSnapshot_t *status);

#endif
