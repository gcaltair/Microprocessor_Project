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
extern PID_Controller pid_speed_left;
extern PID_Controller pid_speed_right;
extern int pwm_output_left,pwm_output_right;

#endif //SYSTEM_H
