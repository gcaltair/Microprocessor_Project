/* pid.h (已修改) */
#ifndef __PID_H
#define __PID_H

#define SPEED_ERROR_DEADBAND 0.01f
#define ANGLE_CONTROL_DEADBAND 1.0f // 原先是2
#define MOTOR_DEAD_ZONE 686
#define POSITION_REACHED_THRESHOLD 0.05f // 到达目标的判断阈值 (2厘米)
#define MAX_BASE_SPEED             0.4f  // 位置控制时允许的最大前进速度 (0.4 m/s)
#define MIN_BASE_SPEED             0.05f // 避免速度过低停转的最小速度
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
float PID_Calculate(PID_Controller *pid, float current_value, float dt);
void Speed_Control_Loop(float dt);
void Angle_Speed_Cascade_Control(float angle_current, float base_speed, float dt);
//void Position_Control_Loop(float target_x, float target_y, float dt); // 新增：位置控制总函数
void Update_Relative_Move_PID(float dt);
void Start_Relative_Move(float dx, float dy);

#endif // __PID_H