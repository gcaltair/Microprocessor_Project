#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "hc04.h"
#include "lidar.h"
#include "motor.h"
#include "navigation_task.h"
#include "pid.h"
#include "robot_app.h"
#include "robot_config.h"

static RobotAppStatus_t s_robotStatus;
static uint32_t s_lastLidarRestartMs = 0U;
static uint32_t s_motorStallStartMs = 0U;
static uint8_t s_motorFaultReported = 0U;
static volatile uint8_t s_estopIrqLatched = 0U;

static void app_lock(void)
{
  if ((osKernelGetState() == osKernelRunning) && (g_appMutex != NULL)) {
    (void)osMutexAcquire(g_appMutex, osWaitForever);
  }
}

static void app_unlock(void)
{
  if ((osKernelGetState() == osKernelRunning) && (g_appMutex != NULL)) {
    (void)osMutexRelease(g_appMutex);
  }
}

static uint8_t estop_pin_active(void)
{
#if (ROBOT_ESTOP_FEATURE_ENABLED != 0U)
  return (HAL_GPIO_ReadPin(ROBOT_ESTOP_GPIO_Port, ROBOT_ESTOP_Pin) == ROBOT_ESTOP_ACTIVE_STATE) ? 1U : 0U;
#else
  return 0U;
#endif
}

static float adc_to_battery_voltage(uint16_t raw)
{
  return ((float)raw / 4095.0f) *
         ROBOT_BATTERY_ADC_FULL_SCALE_V *
         ROBOT_BATTERY_DIVIDER_RATIO;
}

static float pot_to_max_speed(uint16_t raw)
{
  float ratio = (float)raw / 4095.0f;
  return ROBOT_POT_MIN_SPEED_MPS +
         (ratio * (ROBOT_POT_MAX_SPEED_MPS - ROBOT_POT_MIN_SPEED_MPS));
}

static void str_to_upper_copy(char *dst, const char *src, uint16_t dst_size)
{
  uint16_t idx = 0U;

  if ((dst == NULL) || (src == NULL) || (dst_size == 0U)) {
    return;
  }

  while ((src[idx] != '\0') && (idx + 1U < dst_size)) {
    char ch = src[idx];
    if ((ch >= 'a') && (ch <= 'z')) {
      ch = (char)(ch - ('a' - 'A'));
    }
    dst[idx] = ch;
    idx++;
  }
  dst[idx] = '\0';
}

static uint8_t starts_with(const char *text, const char *prefix)
{
  if ((text == NULL) || (prefix == NULL)) {
    return 0U;
  }

  while (*prefix != '\0') {
    if (*text != *prefix) {
      return 0U;
    }
    text++;
    prefix++;
  }

  return 1U;
}

void RobotApp_Init(void)
{
  (void)memset(&s_robotStatus, 0, sizeof(s_robotStatus));
  s_robotStatus.mode = ROBOT_MODE_IDLE;
  s_robotStatus.max_speed_mps = ROBOT_POT_MAX_SPEED_MPS;
  s_robotStatus.last_lidar_update_ms = HAL_GetTick();
  Control_SetMaxBaseSpeed(s_robotStatus.max_speed_mps);
}

void RobotApp_StartExploration(void)
{
  if (estop_pin_active() != 0U) {
    RobotApp_SetEmergencyStop(1U);
    return;
  }

  app_lock();
  if (s_robotStatus.critical_battery_stop != 0U) {
    app_unlock();
    return;
  }
  s_robotStatus.emergency_stop_active = 0U;
  s_robotStatus.mode = ROBOT_MODE_EXPLORING;
  s_robotStatus.motor_fault = 0U;
  s_motorFaultReported = 0U;
  app_unlock();

  Motor_SetEmergencyStop(0U);
  NavigationTask_ClearGoal();
  if (lidar_raw_stream_active == 0U) {
    RPLIDAR_StartRaw();
  }
}

void RobotApp_SoftStop(void)
{
  NavigationTask_ClearGoal();
  Cancel_Relative_Move();
  Control_SetBaseSpeed(0.0f);
  Motor_StopAll();

  app_lock();
  if (s_robotStatus.emergency_stop_active == 0U) {
    s_robotStatus.mode = ROBOT_MODE_IDLE;
  }
  app_unlock();
}

void RobotApp_ReturnHome(void)
{
  if (estop_pin_active() != 0U) {
    RobotApp_SetEmergencyStop(1U);
    return;
  }

  app_lock();
  if ((s_robotStatus.emergency_stop_active != 0U) ||
      (s_robotStatus.critical_battery_stop != 0U)) {
    app_unlock();
    return;
  }
  s_robotStatus.mode = (s_robotStatus.low_battery_warning != 0U) ?
                       ROBOT_MODE_LOW_BATTERY_RETURN :
                       ROBOT_MODE_RETURNING;
  app_unlock();

  Motor_SetEmergencyStop(0U);
  NavigationTask_SetGoal(ROBOT_HOME_GOAL_X_M, ROBOT_HOME_GOAL_Y_M);
}

void RobotApp_SetEmergencyStop(uint8_t active)
{
#if (ROBOT_ESTOP_FEATURE_ENABLED == 0U)
  (void)active;

  app_lock();
  s_robotStatus.emergency_stop_active = 0U;
  if (s_robotStatus.mode == ROBOT_MODE_EMERGENCY_STOP) {
    s_robotStatus.mode = ROBOT_MODE_IDLE;
  }
  app_unlock();

  Motor_SetEmergencyStop(0U);
  return;
#else
  if (active != 0U) {
    Motor_SetEmergencyStop(1U);
    NavigationTask_ClearGoal();
    Cancel_Relative_Move();

    app_lock();
    if (s_robotStatus.emergency_stop_active == 0U) {
      s_robotStatus.estop_count++;
    }
    s_robotStatus.emergency_stop_active = 1U;
    s_robotStatus.mode = ROBOT_MODE_EMERGENCY_STOP;
    app_unlock();
    return;
  }

  RobotApp_ClearEmergencyStopRequest();
#endif
}

void RobotApp_ClearEmergencyStopRequest(void)
{
  if (estop_pin_active() != 0U) {
    return;
  }

  app_lock();
  if (s_robotStatus.critical_battery_stop != 0U) {
    app_unlock();
    return;
  }
  s_robotStatus.emergency_stop_active = 0U;
  if (s_robotStatus.mode == ROBOT_MODE_EMERGENCY_STOP) {
    s_robotStatus.mode = ROBOT_MODE_IDLE;
  }
  app_unlock();

  Motor_SetEmergencyStop(0U);
}

void RobotApp_NotifyEmergencyStopFromIsr(void)
{
  s_estopIrqLatched = 1U;
  Motor_EmergencyStopFromIsr();
}

void RobotApp_SetDisplayPage(uint8_t page)
{
  app_lock();
  s_robotStatus.display_page = (uint8_t)(page % 3U);
  app_unlock();
}

void RobotApp_NextDisplayPage(void)
{
  app_lock();
  s_robotStatus.display_page = (uint8_t)((s_robotStatus.display_page + 1U) % 3U);
  app_unlock();
}

void RobotApp_SetOledReady(uint8_t ready)
{
  app_lock();
  s_robotStatus.oled_ready = (ready != 0U) ? 1U : 0U;
  app_unlock();
}

void RobotApp_RecordButtonEvent(void)
{
  app_lock();
  s_robotStatus.button_event_count++;
  app_unlock();
}

void RobotApp_UpdateAnalogInputs(uint16_t battery_raw, uint16_t potentiometer_raw)
{
  float battery_v = adc_to_battery_voltage(battery_raw);
  float max_speed = pot_to_max_speed(potentiometer_raw);
  uint8_t need_return_home = 0U;
  uint8_t need_critical_stop = 0U;

  Control_SetMaxBaseSpeed(max_speed);

  app_lock();
  s_robotStatus.battery_adc_raw = battery_raw;
  s_robotStatus.potentiometer_adc_raw = potentiometer_raw;
  s_robotStatus.battery_voltage_v = battery_v;
  s_robotStatus.max_speed_mps = max_speed;

  if (battery_v <= ROBOT_BATTERY_CRITICAL_VOLTAGE) {
    if (s_robotStatus.critical_battery_stop == 0U) {
      s_robotStatus.low_battery_count++;
    }
    s_robotStatus.low_battery_warning = 1U;
    s_robotStatus.critical_battery_stop = 1U;
    need_critical_stop = 1U;
  } else if (battery_v <= ROBOT_BATTERY_LOW_VOLTAGE) {
    if (s_robotStatus.low_battery_warning == 0U) {
      s_robotStatus.low_battery_count++;
      need_return_home = 1U;
    }
    s_robotStatus.low_battery_warning = 1U;
    s_robotStatus.critical_battery_stop = 0U;
  } else {
    s_robotStatus.low_battery_warning = 0U;
    s_robotStatus.critical_battery_stop = 0U;
  }
  app_unlock();

  if (need_critical_stop != 0U) {
    RobotApp_SetEmergencyStop(1U);
  } else if (need_return_home != 0U) {
    RobotApp_ReturnHome();
  }
}

void RobotApp_MonitorHealth(const FreertosRuntimeStats_t *runtime_stats)
{
  uint32_t now_ms = HAL_GetTick();
  uint8_t restart_lidar = 0U;
  uint8_t lidar_fault = 0U;
  uint8_t motor_fault = 0U;
  ControlDebugSnapshot_t control;

  if (runtime_stats == NULL) {
    return;
  }

  if (s_estopIrqLatched != 0U) {
    s_estopIrqLatched = 0U;
    RobotApp_SetEmergencyStop(1U);
  }

  if (estop_pin_active() != 0U) {
    RobotApp_SetEmergencyStop(1U);
  }

  app_lock();
  if (runtime_stats->lidar_scan_complete_count != s_robotStatus.last_lidar_scan_count) {
    s_robotStatus.last_lidar_scan_count = runtime_stats->lidar_scan_complete_count;
    s_robotStatus.last_lidar_update_ms = now_ms;
    s_robotStatus.lidar_fault = 0U;
  } else if ((lidar_raw_stream_active != 0U) &&
             ((now_ms - s_robotStatus.last_lidar_update_ms) > ROBOT_LIDAR_TIMEOUT_MS)) {
    lidar_fault = 1U;
    s_robotStatus.lidar_fault = 1U;
    if ((now_ms - s_lastLidarRestartMs) > ROBOT_LIDAR_RESTART_INTERVAL_MS) {
      s_lastLidarRestartMs = now_ms;
      s_robotStatus.fault_recovery_count++;
      restart_lidar = 1U;
    }
  }
  app_unlock();

  if (restart_lidar != 0U) {
    RPLIDAR_StartRaw();
  }

  Control_GetDebugSnapshot(&control);
  if ((((control.left_pwm > ROBOT_MOTOR_STALL_PWM_THRESHOLD) &&
        (fabsf(control.left_speed_feedback_mps) < ROBOT_MOTOR_STALL_SPEED_THRESHOLD)) ||
       ((control.right_pwm > ROBOT_MOTOR_STALL_PWM_THRESHOLD) &&
        (fabsf(control.right_speed_feedback_mps) < ROBOT_MOTOR_STALL_SPEED_THRESHOLD))) &&
      (Motor_IsEmergencyStopped() == 0U)) {
    if (s_motorStallStartMs == 0U) {
      s_motorStallStartMs = now_ms;
    } else if ((now_ms - s_motorStallStartMs) > ROBOT_MOTOR_STALL_TIMEOUT_MS) {
      motor_fault = 1U;
    }
  } else {
    s_motorStallStartMs = 0U;
  }

  if ((motor_fault != 0U) && (s_motorFaultReported == 0U)) {
    s_motorFaultReported = 1U;
    RobotApp_SoftStop();
    app_lock();
    s_robotStatus.motor_fault = 1U;
    s_robotStatus.mode = ROBOT_MODE_FAULT;
    app_unlock();
  } else if (lidar_fault == 0U) {
    app_lock();
    if ((s_robotStatus.mode == ROBOT_MODE_FAULT) && (s_robotStatus.motor_fault == 0U)) {
      s_robotStatus.mode = ROBOT_MODE_IDLE;
    }
    app_unlock();
  }
}

uint8_t RobotApp_ProcessDebugCommand(const char *command)
{
  char upper[80];

  if (command == NULL) {
    return 0U;
  }

  while ((*command == ' ') || (*command == '\t')) {
    command++;
  }

  str_to_upper_copy(upper, command, sizeof(upper));

  app_lock();
  s_robotStatus.debug_command_count++;
  app_unlock();

  if ((starts_with(upper, "START") != 0U) || (starts_with(upper, "RUN") != 0U)) {
    RobotApp_StartExploration();
    uart_printf("OK START\r\n");
    return 1U;
  }

  if ((starts_with(upper, "STOP") != 0U) || (starts_with(upper, "HALT") != 0U)) {
    RobotApp_SoftStop();
    uart_printf("OK STOP\r\n");
    return 1U;
  }

  if ((starts_with(upper, "ESTOP") != 0U) || (starts_with(upper, "E-STOP") != 0U)) {
    RobotApp_SetEmergencyStop(1U);
    uart_printf("OK ESTOP DISABLED\r\n");
    return 1U;
  }

  if ((starts_with(upper, "CLEAR") != 0U) || (starts_with(upper, "RESETSTOP") != 0U)) {
    RobotApp_ClearEmergencyStopRequest();
    uart_printf("OK CLEAR\r\n");
    return 1U;
  }

  if ((starts_with(upper, "HOME") != 0U) || (starts_with(upper, "RETURN") != 0U)) {
    RobotApp_ReturnHome();
    uart_printf("OK HOME\r\n");
    return 1U;
  }

  if (starts_with(upper, "DISP") != 0U) {
    RobotApp_NextDisplayPage();
    uart_printf("OK DISP\r\n");
    return 1U;
  }

  if ((starts_with(upper, "SPD ") != 0U) || (starts_with(upper, "SPEED ") != 0U)) {
    const char *arg = (starts_with(upper, "SPD ") != 0U) ? &upper[4] : &upper[6];
    float speed = strtof(arg, NULL);
    if ((speed >= ROBOT_POT_MIN_SPEED_MPS) && (speed <= ROBOT_POT_MAX_SPEED_MPS)) {
      Control_SetMaxBaseSpeed(speed);
      app_lock();
      s_robotStatus.max_speed_mps = speed;
      app_unlock();
      uart_printf("OK SPD %.3f\r\n", speed);
    } else {
      uart_printf("ERR SPD\r\n");
    }
    return 1U;
  }

  if (starts_with(upper, "PID ") != 0U) {
    char *end_ptr;
    char loop_id = upper[4];
    float kp = strtof(&upper[6], &end_ptr);
    float ki = strtof(end_ptr, &end_ptr);
    float kd = strtof(end_ptr, NULL);
    if (Control_SetPidTunings(loop_id, kp, ki, kd) != 0U) {
      uart_printf("OK PID %c %.4f %.4f %.4f\r\n", loop_id, kp, ki, kd);
    } else {
      uart_printf("ERR PID\r\n");
    }
    return 1U;
  }

  if (starts_with(upper, "WT ") != 0U) {
    char *end_ptr;
    float left = strtof(&upper[3], &end_ptr);
    float right = strtof(end_ptr, NULL);
    if (Control_SetWheelSpeedTest(left, right) != 0U) {
      uart_printf("OK WT %.3f %.3f\r\n", left, right);
    } else {
      uart_printf("ERR WT\r\n");
    }
    return 1U;
  }

  if (starts_with(upper, "STAT") != 0U) {
    RobotAppStatus_t status;
    RobotApp_GetStatusSnapshot(&status);
    uart_printf("STAT mode=%s bat=%.2f max=%.3f estop=%u lidar=%u motor=%u\r\n",
                RobotApp_ModeName(status.mode),
                status.battery_voltage_v,
                status.max_speed_mps,
                status.emergency_stop_active,
                status.lidar_fault,
                status.motor_fault);
    return 1U;
  }

  return 0U;
}

void RobotApp_GetStatusSnapshot(RobotAppStatus_t *status)
{
  if (status == NULL) {
    return;
  }

  app_lock();
  *status = s_robotStatus;
  app_unlock();
}

const char *RobotApp_ModeName(RobotMode_t mode)
{
  switch (mode) {
    case ROBOT_MODE_IDLE:
      return "IDLE";
    case ROBOT_MODE_EXPLORING:
      return "EXP";
    case ROBOT_MODE_RETURNING:
      return "HOME";
    case ROBOT_MODE_EMERGENCY_STOP:
      return "ESTOP";
    case ROBOT_MODE_LOW_BATTERY_RETURN:
      return "LOWBAT";
    case ROBOT_MODE_FAULT:
      return "FAULT";
    default:
      return "UNK";
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if ((GPIO_Pin == ROBOT_ESTOP_Pin) && (estop_pin_active() != 0U)) {
    RobotApp_NotifyEmergencyStopFromIsr();
  }
}
