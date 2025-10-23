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


// 可选累计姿态(若你在别处要用, 否则可以忽略)
extern volatile float g_x;
extern volatile float g_y;

// 过滤后当前轮速(米/秒)
extern float g_left_speed;
extern float g_right_speed;

// odometry.h 或 encoder.h
typedef struct {
    float x;
    float y;
    float theta; // 使用连续角度值
} Pose_t;

// 函数声明
void Odometry_GetPose(Pose_t* pose);



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

#endif /* ENCODER_H */
