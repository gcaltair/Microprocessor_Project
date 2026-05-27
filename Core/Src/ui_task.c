#include <stdio.h>
#include <string.h>

#include "cmsis_os.h"

#include "freertos_app.h"
#include "localization_task.h"
#include "mapping_task.h"
#include "navigation_task.h"
#include "occupancy_grid.h"
#include "oled_ssd1306.h"
#include "pid.h"
#include "robot_app.h"
#include "robot_config.h"

typedef struct {
  GPIO_TypeDef *port;
  uint16_t pin;
  GPIO_PinState active_state;
  uint8_t stable_active;
  uint8_t last_raw_active;
  uint32_t last_change_ms;
  uint32_t pressed_ms;
} UiButton_t;

static UiButton_t s_startButton = {
    .port = B1_GPIO_Port,
    .pin = B1_Pin,
    .active_state = GPIO_PIN_RESET,
};

static uint8_t ui_button_update(UiButton_t *button, uint32_t now_ms, uint32_t *press_duration_ms)
{
  uint8_t raw_active;

  if ((button == NULL) || (press_duration_ms == NULL)) {
    return 0U;
  }

  raw_active = (HAL_GPIO_ReadPin(button->port, button->pin) == button->active_state) ? 1U : 0U;
  if (raw_active != button->last_raw_active) {
    button->last_raw_active = raw_active;
    button->last_change_ms = now_ms;
  }

  if ((now_ms - button->last_change_ms) < ROBOT_BUTTON_DEBOUNCE_MS) {
    return 0U;
  }

  if (raw_active == button->stable_active) {
    return 0U;
  }

  button->stable_active = raw_active;
  if (raw_active != 0U) {
    button->pressed_ms = now_ms;
    return 0U;
  }

  *press_duration_ms = now_ms - button->pressed_ms;
  return 1U;
}

static void ui_handle_start_button(uint32_t press_duration_ms)
{
  RobotAppStatus_t app_status;

  RobotApp_RecordButtonEvent();
  RobotApp_GetStatusSnapshot(&app_status);

  if (press_duration_ms >= ROBOT_BUTTON_LONG_PRESS_MS) {
    RobotApp_ReturnHome();
    return;
  }

  if (app_status.emergency_stop_active != 0U) {
    RobotApp_ClearEmergencyStopRequest();
    RobotApp_StartExploration();
  } else if ((app_status.mode == ROBOT_MODE_IDLE) || (app_status.mode == ROBOT_MODE_FAULT)) {
    RobotApp_StartExploration();
  } else {
    RobotApp_NextDisplayPage();
  }
}

static uint8_t ui_compute_map_progress_percent(void)
{
  MappingGridMeta_t meta;
  uint32_t known_cells = 0U;
  uint32_t cell_count;
  uint16_t x;
  uint16_t y;

  if (MappingTask_GetGridMeta(&meta) == 0U) {
    return 0U;
  }

  cell_count = (uint32_t)meta.width_cells * meta.height_cells;
  if ((cell_count == 0U) || (cell_count > OGM_MAX_CELL_COUNT)) {
    return 0U;
  }

  if (MappingTask_BeginGridRead() == 0U) {
    return 0U;
  }

  for (y = 0U; y < meta.height_cells; ++y) {
    for (x = 0U; x < meta.width_cells; ++x) {
      int8_t cell_value = OGM_UNKNOWN_LOG_ODDS;
      if ((MappingTask_ReadCellDuringGridRead((int16_t)x, (int16_t)y, &cell_value) != 0U) &&
          (cell_value != OGM_UNKNOWN_LOG_ODDS)) {
        known_cells++;
      }
    }
  }
  MappingTask_EndGridRead();

  return (uint8_t)((known_cells * 100U) / cell_count);
}

static void ui_draw_status_page(const RobotAppStatus_t *app_status)
{
  SlamPose2D_t pose;
  NavigationTaskStats_t nav_stats;
  char line[24];
  uint8_t progress;

  LocalizationTask_GetPoseSnapshot(&pose);
  NavigationTask_GetStatsSnapshot(&nav_stats);
  progress = ui_compute_map_progress_percent();

  OLED_Clear();
  (void)snprintf(line, sizeof(line), "MODE %-6s P%u", RobotApp_ModeName(app_status->mode), app_status->display_page);
  OLED_DrawString(0U, 0U, line);
  (void)snprintf(line, sizeof(line), "BAT %.2fV %s",
                 app_status->battery_voltage_v,
                 app_status->low_battery_warning ? "LOW" : "OK");
  OLED_DrawString(0U, 8U, line);
  (void)snprintf(line, sizeof(line), "X% .2f Y% .2f", pose.x_m, pose.y_m);
  OLED_DrawString(0U, 16U, line);
  (void)snprintf(line, sizeof(line), "TH% .1f MAP %u%%", pose.theta_deg, progress);
  OLED_DrawString(0U, 24U, line);
  (void)snprintf(line, sizeof(line), "GOAL %u NAV %u",
                 nav_stats.goal_valid,
                 (uint8_t)nav_stats.last_status);
  OLED_DrawString(0U, 32U, line);
  (void)snprintf(line, sizeof(line), "T % .2f,% .2f",
                 nav_stats.target_pose.x_m,
                 nav_stats.target_pose.y_m);
  OLED_DrawString(0U, 40U, line);
  (void)snprintf(line, sizeof(line), "MAX %.2fm/s", app_status->max_speed_mps);
  OLED_DrawString(0U, 48U, line);
  OLED_DrawString(0U, 56U, "B1 start/page");
  OLED_Update();
}

static void ui_draw_control_page(const RobotAppStatus_t *app_status)
{
  ControlDebugSnapshot_t control;
  NavigationTaskStats_t nav_stats;
  char line[24];

  (void)app_status;
  Control_GetDebugSnapshot(&control);
  NavigationTask_GetStatsSnapshot(&nav_stats);

  OLED_Clear();
  OLED_DrawString(0U, 0U, "CONTROL");
  (void)snprintf(line, sizeof(line), "PWM %u/%u", control.left_pwm, control.right_pwm);
  OLED_DrawString(0U, 8U, line);
  (void)snprintf(line, sizeof(line), "SP %.2f %.2f",
                 control.left_speed_setpoint_mps,
                 control.right_speed_setpoint_mps);
  OLED_DrawString(0U, 16U, line);
  (void)snprintf(line, sizeof(line), "FB %.2f %.2f",
                 control.left_speed_feedback_mps,
                 control.right_speed_feedback_mps);
  OLED_DrawString(0U, 24U, line);
  (void)snprintf(line, sizeof(line), "ERR A%.1f P%.2f",
                 control.angle_error_deg,
                 control.position_error_m);
  OLED_DrawString(0U, 32U, line);
  (void)snprintf(line, sizeof(line), "NAV P%u S%u",
                 (uint8_t)nav_stats.phase,
                 (uint8_t)nav_stats.last_status);
  OLED_DrawString(0U, 40U, line);
  (void)snprintf(line, sizeof(line), "DIST %.2fm", nav_stats.distance_to_goal_m);
  OLED_DrawString(0U, 48U, line);
  OLED_DrawString(0U, 56U, "B1 page/home");
  OLED_Update();
}

static void ui_draw_diagnostics_page(const RobotAppStatus_t *app_status)
{
  FreertosRuntimeStats_t runtime;
  char line[24];

  Freertos_GetRuntimeStatsSnapshot(&runtime);

  OLED_Clear();
  OLED_DrawString(0U, 0U, "DIAGNOSTICS");
  (void)snprintf(line, sizeof(line), "LIDAR %lu",
                 (unsigned long)runtime.lidar_scan_complete_count);
  OLED_DrawString(0U, 8U, line);
  (void)snprintf(line, sizeof(line), "DROP %lu REC %lu",
                 (unsigned long)runtime.lidar_dma_drop_count,
                 (unsigned long)app_status->fault_recovery_count);
  OLED_DrawString(0U, 16U, line);
  (void)snprintf(line, sizeof(line), "HEAP %lu",
                 (unsigned long)runtime.free_heap_bytes);
  OLED_DrawString(0U, 24U, line);
  (void)snprintf(line, sizeof(line), "REC %lu",
                 (unsigned long)app_status->fault_recovery_count);
  OLED_DrawString(0U, 32U, line);
  (void)snprintf(line, sizeof(line), "ADC %u %u",
                 app_status->battery_adc_raw,
                 app_status->potentiometer_adc_raw);
  OLED_DrawString(0U, 40U, line);
  (void)snprintf(line, sizeof(line), "BTN %lu CMD %lu",
                 (unsigned long)app_status->button_event_count,
                 (unsigned long)app_status->debug_command_count);
  OLED_DrawString(0U, 48U, line);
  OLED_DrawString(0U, 56U, app_status->motor_fault ? "MOTOR FAULT" : "FAULTS CLEAR");
  OLED_Update();
}

static void ui_draw_current_page(void)
{
  RobotAppStatus_t app_status;

  RobotApp_GetStatusSnapshot(&app_status);
  if (app_status.display_page == 0U) {
    ui_draw_status_page(&app_status);
  } else if (app_status.display_page == 1U) {
    ui_draw_control_page(&app_status);
  } else {
    ui_draw_diagnostics_page(&app_status);
  }
}

void StartUiTask(void *argument)
{
  uint32_t last_oled_refresh_ms = 0U;
  uint32_t last_oled_probe_ms = 0U;

  (void)argument;
  RobotApp_SetOledReady(OLED_Init());

  for (;;) {
    uint32_t now_ms = HAL_GetTick();
    uint32_t press_duration_ms = 0U;

    if (ui_button_update(&s_startButton, now_ms, &press_duration_ms) != 0U) {
      ui_handle_start_button(press_duration_ms);
    }

    if ((OLED_IsReady() == 0U) && ((now_ms - last_oled_probe_ms) >= 1000U)) {
      last_oled_probe_ms = now_ms;
      RobotApp_SetOledReady(OLED_Init());
    }

    if ((OLED_IsReady() != 0U) && ((now_ms - last_oled_refresh_ms) >= ROBOT_OLED_REFRESH_MS)) {
      last_oled_refresh_ms = now_ms;
      ui_draw_current_page();
    }

    osDelay(ROBOT_UI_TASK_PERIOD_MS);
  }
}
