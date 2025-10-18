#include "system.h"
#include "encoder.h"
#include <math.h>

// 仅本文件使用的速度换算系数
#define PULSE_TO_SPEED_FACTOR (PI * DIAMETER / ENCODER_PULSES_PER_REV / ENCODER_SAMPLING_PERIOD)

// 滤波参数
#define MAX_REASONABLE_SPEED 12.0f
#define SPEED_FILTER_ALPHA   0.85f

// 全局变量定义(头文件中为 extern)
volatile float g_dl_acc = 0.0f;
volatile float g_dr_acc = 0.0f;
volatile float g_dth_acc = 0.0f;
volatile float g_x = 0.0f;
volatile float g_y = 0.0f;
volatile float g_th = 0.0f;
float g_left_speed = 0.0f;
float g_right_speed = 0.0f;

// 上次计数
static int16_t last_left_count = 0;
static int16_t last_right_count = 0;

/**
 * @brief 编码器初始化
 */
void encoder_init()
{
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    last_left_count  = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
    last_right_count = -(int16_t)__HAL_TIM_GET_COUNTER(&htim1);
}

/**
 * @brief 更新编码器速度 (核心修改部分 - 已集成滤波)
 * @note  1. 采用不清零的差分法
 * @note  2. 增加了限幅和一阶低通滤波
 */
void encoder_update_speed(void) {
    int16_t current_left_count  = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
    int16_t current_right_count = -(int16_t)__HAL_TIM_GET_COUNTER(&htim1);

    int16_t left_pulse_delta  = current_left_count  - last_left_count;
    int16_t right_pulse_delta = current_right_count - last_right_count;

    last_left_count  = current_left_count;
    last_right_count = current_right_count;

    float raw_left_speed  = left_pulse_delta  * PULSE_TO_SPEED_FACTOR;
    float raw_right_speed = right_pulse_delta * PULSE_TO_SPEED_FACTOR;

    if (fabsf(raw_left_speed)  > MAX_REASONABLE_SPEED)  raw_left_speed  = g_left_speed;
    if (fabsf(raw_right_speed) > MAX_REASONABLE_SPEED)  raw_right_speed = g_right_speed;

    g_left_speed  = (1.0f - SPEED_FILTER_ALPHA) * g_left_speed  + SPEED_FILTER_ALPHA * raw_left_speed;
    g_right_speed = (1.0f - SPEED_FILTER_ALPHA) * g_right_speed + SPEED_FILTER_ALPHA * raw_right_speed;

    // 位移增量(单位: 米). 正: 轮按前进方向转动; 负: 反向
    float dl = left_pulse_delta  * PULSE_TO_DIST_FACTOR;
    float dr = right_pulse_delta * PULSE_TO_DIST_FACTOR;

    // 航向角增量 dδ (单位: 弧度). dδ = (dr - dl)/轮距
    // 右轮比左轮前进多 => 车体逆时针 => dδ 为正
    float dth = (dr - dl) / WHEEL_BASE;

    g_dl_acc  += dl;
    g_dr_acc  += dr;
    g_dth_acc += dth;
}

/**
 * @brief 复位编码器计数
 */
void encoder_Reset(void) {
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    last_left_count = 0;
    last_right_count = 0;
    g_left_speed = 0;
    g_right_speed = 0;
    g_dl_acc = g_dr_acc = g_dth_acc = 0.0f;
}

/**
 * @brief 获取左轮编码器原始计数值
 * @retval 计数值
 */
uint32_t encoder_left_get_count(void)  { return __HAL_TIM_GET_COUNTER(&htim1); }

/**
 * @brief 获取右轮编码器原始计数值
 * @retval 计数值
 */
uint32_t encoder_right_get_count(void) { return __HAL_TIM_GET_COUNTER(&htim2); }

// 读出自上次查询以来的增量并清零
void odom_pop_delta(float *dl, float *dr, float *dth)
{
    __disable_irq();
    *dl  = g_dl_acc;
    *dr  = g_dr_acc;
    *dth = g_dth_acc;
    g_dl_acc = g_dr_acc = g_dth_acc = 0.0f;
    __enable_irq();
}


// 全局变量:系统更新标志,使用计时器更新
bool g_system_update_flag = false; 

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // 假设TIM4是你的周期性任务定时器 (例如10ms或20ms)
    if (htim->Instance == htim4.Instance)
    {
        // 这个标志位可以用来在主循环中触发 encoder_update_speed() 和其他控制任务
        g_system_update_flag = true;
    }
}