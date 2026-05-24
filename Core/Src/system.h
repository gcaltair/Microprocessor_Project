#ifndef SYSTEM_H
#define SYSTEM_H

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "../Inc/slam_types.h"
#include "../Inc/tim.h"
#include "../Inc/usart.h"
#include "MPU6500.h"
#include "encoder.h"
#include "hc04.h"
#include "lidar.h"
#include "motor.h"
#include "pid.h"

extern volatile float g_left_speed;
extern volatile float g_right_speed;
extern AccelData g_accel_data;
extern GyroData g_gyro_data;

extern volatile PID_Controller g_pid_speed_left;
extern volatile PID_Controller g_pid_speed_right;
extern volatile PID_Controller g_pid_position;
extern volatile PID_Controller g_pid_angle;

extern volatile uint32_t overflow_count;
extern volatile float base_car_speed;

typedef enum {
    CONTROL_MODE_MANUAL = 0,
    CONTROL_MODE_POSITION = 1,
    CONTROL_MODE_SPEED_TEST = 2
} ControlMode;

typedef enum {
    RELATIVE_MOVE_IDLE = 0,
    RELATIVE_MOVE_TURNING = 1,
    RELATIVE_MOVE_DRIVING = 2,
} RelativeMoveState;

extern volatile RelativeMoveState g_relative_move_state;
extern volatile ControlMode g_control_mode;

#endif /* SYSTEM_H */
