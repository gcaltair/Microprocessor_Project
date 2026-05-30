#ifndef ROBOT_APP_H
#define ROBOT_APP_H

#include <stdint.h>

#include "freertos_app.h"

typedef enum {
  ROBOT_MODE_IDLE = 0,
  ROBOT_MODE_EXPLORING = 1,
  ROBOT_MODE_RETURNING = 2,
  ROBOT_MODE_EMERGENCY_STOP = 3,
  ROBOT_MODE_LOW_BATTERY_RETURN = 4,
  ROBOT_MODE_FAULT = 5
} RobotMode_t;

typedef struct {
  RobotMode_t mode;
  uint8_t emergency_stop_active;
  uint8_t low_battery_warning;
  uint8_t critical_battery_stop;
  uint8_t lidar_fault;
  uint8_t motor_fault;
  uint8_t oled_ready;
  uint8_t display_page;
  uint16_t battery_adc_raw;
  uint16_t potentiometer_adc_raw;
  float battery_voltage_v;
  float max_speed_mps;
  uint32_t estop_count;
  uint32_t low_battery_count;
  uint32_t fault_recovery_count;
  uint32_t button_event_count;
  uint32_t debug_command_count;
  uint32_t last_lidar_scan_count;
  uint32_t last_lidar_update_ms;
} RobotAppStatus_t;

void RobotApp_Init(void);
void RobotApp_StartExploration(void);
void RobotApp_SoftStop(void);
void RobotApp_ReturnHome(void);
void RobotApp_SetEmergencyStop(uint8_t active);
void RobotApp_ClearEmergencyStopRequest(void);
void RobotApp_NotifyEmergencyStopFromIsr(void);
void RobotApp_SetDisplayPage(uint8_t page);
void RobotApp_NextDisplayPage(void);
void RobotApp_SetOledReady(uint8_t ready);
void RobotApp_RecordButtonEvent(void);
void RobotApp_UpdateAnalogInputs(uint16_t battery_raw, uint16_t potentiometer_raw);
void RobotApp_MonitorHealth(const FreertosRuntimeStats_t *runtime_stats);
uint8_t RobotApp_ProcessDebugCommand(const char *command);
void RobotApp_GetStatusSnapshot(RobotAppStatus_t *status);
const char *RobotApp_ModeName(RobotMode_t mode);

#endif /* ROBOT_APP_H */
