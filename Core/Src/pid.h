/* pid.h (已修改) */
#ifndef __PID_H
#define __PID_H

/*
 * @brief PID控制器结构体
 * (结构体内部保持float，以保证增益和积分的精度)
 */
typedef struct
{
    float Kp;
    float Ki;
    float Kd;

    float setpoint;     // 目标值 (仍然使用float以支持非整数目标)

    float integral;
    float last_error;

    float output_min;
    float output_max;
    float integral_max;

} PID_Controller;

/* 函数声明 */

void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float out_min, float out_max);

/**
 * @brief  计算PID控制器的输出 (修改版)
 * @param  pid: 指向PID_Controller结构体的指针
 * @param  current_value: 当前的测量值 (int类型)
 * @retval 计算得到的PID输出值 (int类型)
 */
int PID_Calculate(PID_Controller *pid, float current_value);

void Speed_Control_Loop(void);


#endif // __PID_H