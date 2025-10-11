//
// Created by G on 25-7-2.
//

#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

// 初始化编码器
void encoder_init(void);

// 获取编码器计数
uint32_t encoder_left_get_count(void);
uint32_t encoder_right_get_count(void);

// 检测编码器溢出
void calculate_diffA(void);
void calculate_diffB(void);

// 获取总计数（包括溢出计数）
int32_t encoderA_GetTotalCount(void);
int32_t encoderB_GetTotalCount(void);

// 计算编码器速度
float encoderA_CalculateSpeed(uint32_t timeIntervalMs);
float encoderB_CalculateSpeed(uint32_t timeIntervalMs);

// 获取当前速度
float encoderA_GetSpeed(void);
float encoderB_GetSpeed(void);

// 更新两个编码器的速度（需定时调用）
void encoder_update_speed(void);

// 基于SysTick的周期性速度更新（在主循环中调用）
uint8_t encoder_UpdateSpeed_SysTick(void);

// 重置编码器
void encoder_Reset(void);

#endif /* ENCODER_H */
