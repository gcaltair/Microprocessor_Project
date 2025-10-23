//
// Created by 11854 on 25-9-30.
//

#ifndef SYSTEM_H
#define SYSTEM_H

#include "stdbool.h"
#include <stdio.h>
#include <string.h>

#include "../Inc/tim.h"
#include "motor.h"
#include "hc04.h"
#include "encoder.h"
#include "MPU6500.h"
#include "pid.h"
#include "lidar.h"
#include "MPU6500.h"
#include "../Inc/usart.h"

/*全局变量区*/
extern bool g_system_update_flag;
extern float g_left_speed;
extern float g_right_speed;
extern AccelData g_accel_data;
extern GyroData g_gyro_data;

extern volatile PID_Controller g_pid_speed_left;
extern volatile PID_Controller g_pid_speed_right;
extern volatile PID_Controller g_pid_position;

extern volatile int pwm_output_left,pwm_output_right;
extern volatile uint32_t overflow_count;
extern volatile PID_Controller g_pid_angle;
extern volatile float base_car_speed;
extern volatile float g_dl_acc;   // 左轮累计位移增量(m)
extern volatile float g_dr_acc;   // 右轮累计位移增量(m)
extern volatile float g_dth_acc;  // 航向角增量(rad)

extern volatile float g_target_x;
extern volatile float g_target_y;
extern volatile float g_th;
extern volatile float g_th_continuous; // 【新增】连续累加的绝对角度，给PID使用

typedef enum {
    CONTROL_MODE_MANUAL,   // 手动遥控模式 (F, B, L, R, S)
    CONTROL_MODE_POSITION  // 自动位置控制模式 (P 命令)
} ControlMode;
typedef enum {
    RELATIVE_MOVE_IDLE,      // 空闲状态，等待新任务
    RELATIVE_MOVE_TURNING,   // 正在转向
    RELATIVE_MOVE_DRIVING    // 正在直行
} RelativeMoveState;
extern volatile RelativeMoveState g_relative_move_state;
extern volatile ControlMode g_control_mode;

extern volatile uint8_t scan_data_ready_flag;
extern LidarPoint_t lidar_points[];
extern volatile uint16_t point_count;
#endif //SYSTEM_H
