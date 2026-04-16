//
// Created by G on 25-7-2.
//

#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>
#include "../Inc/slam_types.h"

// 编码器/机械参数
#define ENCODER_PULSES_PER_REV  380
#define DIAMETER                0.065f
#define PI                      3.14159265358979323846f
#define WHEEL_BASE              0.165f
#define ENCODER_SAMPLING_PERIOD 0.01f

// 位移换算系数: 每个脉冲对应的线位移(米)
#define PULSE_TO_DIST_FACTOR    (PI * DIAMETER / ENCODER_PULSES_PER_REV)


// 可选累计姿态(若你在别处要用, 否则可以忽略)
extern volatile float g_x;
extern volatile float g_y;

// 过滤后当前轮速(米/秒)
extern volatile float g_left_speed;
extern volatile float g_right_speed;
extern volatile float g_encoder_left_forward_scale;
extern volatile float g_encoder_left_reverse_scale;
extern volatile float g_encoder_right_forward_scale;
extern volatile float g_encoder_right_reverse_scale;


// 初始化编码器
void encoder_init(void);

// 获取编码器原始计数
uint32_t encoder_left_get_count(void);
uint32_t encoder_right_get_count(void);

// 更新速度与里程计增量 (需在固定周期调用)
void encoder_update_speed(void);

// 复位
void encoder_Reset(void);

void Odometry_Update(float dt);
void Odometry_GetPoseSnapshot(SlamPose2D_t *pose);
void Encoder_GetCalibration(float *left_forward, float *left_reverse, float *right_forward, float *right_reverse);
uint8_t Encoder_SetCalibration(float left_forward, float left_reverse, float right_forward, float right_reverse);

#endif /* ENCODER_H */
