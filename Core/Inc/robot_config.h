#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include "main.h"

#define ROBOT_BUTTON_DEBOUNCE_MS             35U
#define ROBOT_BUTTON_LONG_PRESS_MS           1000U
#define ROBOT_UI_TASK_PERIOD_MS              50U
#define ROBOT_OLED_REFRESH_MS                250U

#define ROBOT_ESTOP_Pin                      GPIO_PIN_2
#define ROBOT_ESTOP_GPIO_Port                GPIOB
#define ROBOT_ESTOP_ACTIVE_STATE             GPIO_PIN_RESET
#define ROBOT_ESTOP_FEATURE_ENABLED          0U

#define ROBOT_BATTERY_ADC_FULL_SCALE_V       3.30f
#define ROBOT_BATTERY_DIVIDER_RATIO          3.00f
#define ROBOT_BATTERY_LOW_VOLTAGE            7.00f
#define ROBOT_BATTERY_CRITICAL_VOLTAGE       6.60f

#define ROBOT_POT_MIN_SPEED_MPS              0.06f
#define ROBOT_POT_MAX_SPEED_MPS              0.22f

#define ROBOT_LIDAR_TIMEOUT_MS               3000U
#define ROBOT_LIDAR_RESTART_INTERVAL_MS      5000U
#define ROBOT_MOTOR_STALL_TIMEOUT_MS         900U
#define ROBOT_MOTOR_STALL_PWM_THRESHOLD      1200U
#define ROBOT_MOTOR_STALL_SPEED_THRESHOLD    0.01f

#define ROBOT_HOME_GOAL_X_M                  0.0f
#define ROBOT_HOME_GOAL_Y_M                  0.0f

#endif /* ROBOT_CONFIG_H */
