//
// Created by G on 25-7-2.
//

#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>
#include "../Inc/slam_types.h"

// 编码器/机械参数
#define ENCODER_PULSES_PER_REV  390
#define DIAMETER                0.065f
#define PI                      3.14159265358979323846f
#define WHEEL_BASE              0.165f
#define ENCODER_SAMPLING_PERIOD 0.01f

// 位移换算系数: 每个脉冲对应的线位移(米)
#define PULSE_TO_DIST_FACTOR    (PI * DIAMETER / ENCODER_PULSES_PER_REV)


// 可选累计姿态(若你在别处要用, 否则可以忽略)
extern volatile float g_x;  // 对外提供的里程计 x 坐标估计，单位为米。
extern volatile float g_y;  // 对外提供的里程计 y 坐标估计，单位为米。

// 过滤后当前轮速(米/秒)
extern volatile float g_left_speed;   // 低通滤波后的左轮速度，单位 m/s。
extern volatile float g_right_speed;  // 低通滤波后的右轮速度，单位 m/s。

typedef struct
{
    int16_t left_pulse_delta;   // 最近一次左编码器计数增量。
    int16_t right_pulse_delta;  // 最近一次右编码器计数增量。
    int16_t left_counter_raw;   // 按固件方向约定处理后的左编码器有符号计数。
    int16_t right_counter_raw;  // 按固件方向约定处理后的右编码器有符号计数。
    float raw_left_speed_mps;   // 滤波前的左轮瞬时速度，单位 m/s。
    float raw_right_speed_mps;  // 滤波前的右轮瞬时速度，单位 m/s。
    float odom_left_speed_mps;   // 应用里程计标定比例后的左轮速度。
    float odom_right_speed_mps;  // 应用里程计标定比例后的右轮速度。
} EncoderDebugSnapshot_t;


// 初始化编码器
void encoder_init(void);

// 获取编码器原始计数
uint32_t encoder_left_get_count(void);
uint32_t encoder_right_get_count(void);

// 更新速度与里程计增量 (需在固定周期调用)
void encoder_update_speed(void);

// 复位
void encoder_Reset(void);
void Odometry_ResetPose(void);

void Odometry_Update(float dt);
void Odometry_GetPoseSnapshot(SlamPose2D_t *pose);
void Encoder_GetTravelSnapshot(float *left_distance_m,
                               float *right_distance_m,
                               int16_t *left_counter_raw,
                               int16_t *right_counter_raw);
void Encoder_GetDebugSnapshot(EncoderDebugSnapshot_t *snapshot);

#endif /* ENCODER_H */
