//
// Created by G on 25-7-2.
//
#include "tim.h"

#include "encoder.h"
#include "hc04.h"
// 编码器参数定义
#define ENCODER_PULSES_PER_REV  330     // 编码器每转脉冲数
#define ENCODER_SAMPLING_PERIOD 100     // 采样周期(ms)

// 存储上次编码器计数值
static uint32_t lastCountA = 0;
static uint32_t lastCountB = 0;

// 存储速度计算结果
static float speedA_rpm = 0;
static float speedB_rpm = 0;

// 溢出计数
static int32_t overflowCountA = 0;
static int32_t overflowCountB = 0;

// 用于SysTick计时的变量
static uint32_t lastUpdateTick = 0;

static int32_t countDiffA=0;
static int32_t countDiffB=0;


void encoder_init()
{
    HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL); //启动两个定时器(计数器)

    // 初始化计数值
    lastCountA = __HAL_TIM_GET_COUNTER(&htim1);
    lastCountB = __HAL_TIM_GET_COUNTER(&htim2);
    
    // 初始化SysTick计时
    lastUpdateTick = HAL_GetTick();
    
    // 初始化速度为0
    speedA_rpm = 0;
    speedB_rpm = 0;
    
    // 初始化溢出计数为0
    overflowCountA = 0;
    overflowCountB = 0;
}
// 读取TIM1当前计数值
uint32_t encoderA_GetCount(void) {
    return __HAL_TIM_GET_COUNTER(&htim1);
}
// 读取TIM2当前计数值
uint32_t encoderB_GetCount(void) {
    return __HAL_TIM_GET_COUNTER(&htim2);
}


// 检测并处理编码器A溢出
void calculate_diffA(void) {
    uint32_t currentCount = encoderA_GetCount();

    // 检测上溢
    if (currentCount < 16384 && lastCountA > 49152) {
        countDiffA=65535-lastCountA+currentCount;
    }
        // 检测下溢
    else if (currentCount > 49152 && lastCountA < 16384) {
        countDiffA=currentCount-lastCountA-65535;
    }
    else
    {
        countDiffA=currentCount-lastCountA;
    }
    lastCountA=currentCount;

}

// 检测并处理编码器B溢出
void calculate_diffB(void) {
    uint32_t currentCount = encoderB_GetCount();
    
    // 检测上溢
    if (currentCount < 16384 && lastCountB > 49152) {
        countDiffB=65535-lastCountB+currentCount;
    }
    // 检测下溢
    else if (currentCount > 49152 && lastCountB < 16384) {
       countDiffB=currentCount-lastCountB-65535;
    }
    else
    {
        countDiffB=currentCount-lastCountB;
    }
    lastCountB=currentCount;

}

// 计算编码器A的速度(RPM)
float encoderA_CalculateSpeed(uint32_t timeIntervalMs) {

//    if (timeIntervalMs == 0) {
//        // 防止除零错误
//        speedA_rpm = 0;
//    } else {
//        // 计算RPM: (计数差/每圈脉冲数) * (60秒/时间间隔秒)
//        // 确保使用浮点数计算以避免整数除法截断
//        speedA_rpm = ((float)countDiff * 60000.0f) / ((float)ENCODER_PULSES_PER_REV * (float)timeIntervalMs);
//    }
//
//    // 保存当前计数值，用于下次计算

//    // 返回绝对值，我们只关心速度大小而非方向
//    return (speedA_rpm >= 0) ? speedA_rpm : -speedA_rpm;
    speedA_rpm=countDiffA;
}

// 计算编码器B的速度(RPM)
float encoderB_CalculateSpeed(uint32_t timeIntervalMs) {
    
//    if (timeIntervalMs == 0) {
//        // 防止除零错误
//        speedB_rpm = 0;
//    } else {
//        // 计算RPM: (计数差/每圈脉冲数) * (60秒/时间间隔秒)
//        // 确保使用浮点数计算以避免整数除法截断
//        speedB_rpm = ((float)countDiff * 60000.0f) / ((float)ENCODER_PULSES_PER_REV * (float)timeIntervalMs);
//    }
//
//    // 保存当前计数值，用于下次计算
//    lastTotalCountB = currentTotalCount;
//
//    // 返回绝对值，我们只关心速度大小而非方向
//    return (speedB_rpm >= 0) ? speedB_rpm : -speedB_rpm;
    speedB_rpm=countDiffB;
}

// 获取编码器A的当前速度(RPM)
float encoderA_GetSpeed(void) {
    return speedA_rpm;
}

// 获取编码器B的当前速度(RPM)
float encoderB_GetSpeed(void) {
    return speedB_rpm;
}

// 更新两个编码器的速度 - 在定时器中断中定期调用此函数
void encoder_UpdateSpeed(void) {
    // 检查溢出
    calculate_diffA();
    calculate_diffB();
    
    // 计算速度
    encoderA_CalculateSpeed(ENCODER_SAMPLING_PERIOD);
    encoderB_CalculateSpeed(ENCODER_SAMPLING_PERIOD);
}

// 基于SysTick的周期性速度更新 - 在主循环中调用
// 返回1表示已更新速度，返回0表示尚未到更新时间
uint8_t encoder_UpdateSpeed_SysTick(void) {
    uint32_t currentTick = HAL_GetTick();
    
    // 检查是否已经过了采样周期
    if ((currentTick - lastUpdateTick) >= 200) {
         // 检查溢出
         char msg[50];
         sprintf(msg,"lastCountA: %d, lastCountB: %d\r\n",lastCountA,lastCountB);
         transmit(msg);

        calculate_diffA();
        calculate_diffB();

         // 计算速度
        encoderA_CalculateSpeed(currentTick - lastUpdateTick);
        encoderB_CalculateSpeed(currentTick - lastUpdateTick);

        lastUpdateTick = currentTick;
        return 1;
    }
    
    return 0;
}

// 重置编码器计数和速度
void encoder_Reset(void) {
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    
    lastCountA = 0;
    lastCountB = 0;
    overflowCountA = 0;
    overflowCountB = 0;
    speedA_rpm = 0;
    speedB_rpm = 0;

}
