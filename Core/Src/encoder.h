//
// Created by G on 25-7-2.
//

#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

// 编码器/机械参数
#define ENCODER_PULSES_PER_REV  380
#define DIAMETER                0.065f
#define PI                      3.14159265358979323846f
#define WHEEL_BASE              0.165f
#define ENCODER_SAMPLING_PERIOD 0.01f

// 位移换算系数: 每个脉冲对应的线位移(米)
#define PULSE_TO_DIST_FACTOR    (PI * DIAMETER / ENCODER_PULSES_PER_REV)

// 里程计累计增量(由 encoder_update_speed 周期更新, odom_pop_delta 读取后清零)
extern volatile float g_dl_acc;   // 左轮累计位移增量(m)
extern volatile float g_dr_acc;   // 右轮累计位移增量(m)
extern volatile float g_dth_acc;  // 航向角增量(rad)

// 可选累计姿态(若你在别处要用, 否则可以忽略)
extern volatile float g_x;
extern volatile float g_y;
extern volatile float g_th;

// 过滤后当前轮速(米/秒)
extern float g_left_speed;
extern float g_right_speed;

// 获取并清零增量 (返回 dl, dr, dδ)，单位: m, m, rad
void odom_pop_delta(float *dl, float *dr, float *dth);

// 初始化编码器
void encoder_init(void);

// 获取编码器原始计数
uint32_t encoder_left_get_count(void);
uint32_t encoder_right_get_count(void);

// 更新速度与里程计增量 (需在固定周期调用)
void encoder_update_speed(void);

// 复位
void encoder_Reset(void);

// 基于SysTick的周期性速度更新（在主循环中调用）
uint8_t encoder_UpdateSpeed_SysTick(void);

#endif /* ENCODER_H */
