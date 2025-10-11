#include "system.h"
#include <math.h> // 为了使用 fabs 函数

// 编码器参数定义
#define ENCODER_PULSES_PER_REV  380     // 编码器每转脉冲数
#define ENCODER_SAMPLING_PERIOD 0.01f   // 采样周期(s), 假设你的更新定时器是10ms
#define PI 3.14159265358979323846f
#define DIAMETER 0.065f
#define PULSE_TO_SPEED_FACTOR (PI * DIAMETER / ENCODER_PULSES_PER_REV / ENCODER_SAMPLING_PERIOD)

//-------------------- 新增的滤波参数定义 --------------------
// 限幅滤波阈值 (单位: m/s) - 根据你的小车实际最高速来设置，留一些余量
// 你的最高速是 10m/s, 但这是一个异常值, 假设正常最高速在 2m/s 左右
#define MAX_REASONABLE_SPEED 12.0f

// 一阶低通滤波系数 (alpha) - 取值范围 0.0 ~ 1.0
// alpha 越小, 滤波效果越强(越平滑), 但响应越慢
// alpha 越大, 响应越快, 但滤波效果越弱
// 建议从 0.2 开始尝试
#define SPEED_FILTER_ALPHA 0.85f
//------------------------------------------------------------


// --- 修改了全局速度变量的定义 ---
// 将原来的 g_left_speed 和 g_right_speed 用于存储滤波后的最终速度
float g_left_speed = 0;
float g_right_speed = 0;

// 新增: 用于保存上一次读取的计数值
static int16_t last_left_count = 0;
static int16_t last_right_count = 0;

/**
 * @brief 编码器初始化
 */
void encoder_init()
{
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

    // 初始化 "上一次计数值"，防止第一次计算速度时出错
    last_left_count = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
    last_right_count = -(int16_t)__HAL_TIM_GET_COUNTER(&htim1);
}

/**
 * @brief 更新编码器速度 (核心修改部分 - 已集成滤波)
 * @note  1. 采用不清零的差分法
 * @note  2. 增加了限幅和一阶低通滤波
 */
void encoder_update_speed(void) {
    // 1. 读取当前硬件计数器的总计数值
    int16_t current_left_count = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
    int16_t current_right_count = -(int16_t)__HAL_TIM_GET_COUNTER(&htim1);

    // 2. 计算两次采样之间的脉冲变化量（差值）
    int16_t left_pulse_delta = current_left_count - last_left_count;
    int16_t right_pulse_delta = current_right_count - last_right_count;

    // 3. 更新 "上一次计数值" 为 "当前计数值"，为下一次计算做准备
    last_left_count = current_left_count;
    last_right_count = current_right_count;

    // 4. 计算出当前周期的 "原始速度" (未滤波)
    float raw_left_speed = left_pulse_delta * PULSE_TO_SPEED_FACTOR;
    float raw_right_speed = right_pulse_delta * PULSE_TO_SPEED_FACTOR;

    // 步骤 A: 限幅滤波
    // 如果计算出的瞬时速度绝对值超过了设定的最大合理速度，就认为是异常值
    // 直接使用上一次滤波后的速度值替代本次的异常值
    if(fabs(raw_left_speed) > MAX_REASONABLE_SPEED) {
        raw_left_speed = g_left_speed; // 使用上一次的速度值
    }
    if(fabs(raw_right_speed) > MAX_REASONABLE_SPEED) {
        raw_right_speed = g_right_speed; // 使用上一次的速度值
    }

    // 步骤 B: 一阶低通滤波
    // 公式: 本次滤波后速度 = (1 - alpha) * 上次滤波后速度 + alpha * 本次(限幅后)原始速度
    g_left_speed = (1.0f - SPEED_FILTER_ALPHA) * g_left_speed + SPEED_FILTER_ALPHA * raw_left_speed;
    g_right_speed = (1.0f - SPEED_FILTER_ALPHA) * g_right_speed + SPEED_FILTER_ALPHA * raw_right_speed;

    // ------------------------------------
}

/**
 * @brief 复位编码器计数
 */
void encoder_Reset(void) {
    // 硬件计数器清零
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    __HAL_TIM_SET_COUNTER(&htim2, 0);

    // 同时必须重置 "上一次计数值" 和 "速度" 变量
    last_left_count = 0;
    last_right_count = 0;
    g_left_speed = 0;
    g_right_speed = 0;
}

// ... 其他函数 (encoder_left_get_count, encoder_right_get_count) 保持不变 ...
// ... 你原来的主循环和定时器回调函数也无需修改 ...
/**
 * @brief 获取左轮编码器原始计数值
 * @retval 计数值
 */
uint32_t encoder_left_get_count(void) {
    // 注意：这里的右轮是htim2，左轮是htim1，根据你的接线
    // 但为了与你的原始代码保持一致，这里返回 htim1
    return __HAL_TIM_GET_COUNTER(&htim1);
}

/**
 * @brief 获取右轮编码器原始计数值
 * @retval 计数值
 */
uint32_t encoder_right_get_count(void) {
    return __HAL_TIM_GET_COUNTER(&htim2);
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