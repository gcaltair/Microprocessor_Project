/* pid.c (已修改) */
#include <math.h>

#include"system.h"
/**
 * @brief 电机死区（启动阈值）补偿值
 * @note  这个值需要你通过实验来精确确定！
 * @note  方法：在无PID的情况下，手动增加PWM，找到能让车轮“刚好”稳定转动的最小PWM值。
 * @note  根据你之前的数据，这个值在43左右，你可以先从40或43开始尝试。
 */
#define MOTOR_DEAD_ZONE 38
/* 定义左右轮速度环的PID控制器实例 */
PID_Controller g_pid_speed_left;
PID_Controller g_pid_speed_right;
PID_Controller g_pid_angle;
volatile float base_car_speed = 0.0f;
// PID_Init 函数保持不变，它使用float进行初始化
void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float out_min, float out_max)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->setpoint = 0.0f;
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->output_min = out_min;
    pid->output_max = out_max;
    pid->integral_max = out_max;
}

/**
 * @brief  计算PID控制器的输出 (修改版)
 * @param  pid: 指向PID_Controller结构体的指针
 * @param  current_value: 当前的测量值 (int类型)
 * @retval 计算得到的PID输出值 (int类型)
 */
int PID_Calculate(PID_Controller *pid, float current_value)
{
    float error, p_out, i_out, d_out, output_float;

    // 1. 计算误差。将输入转换为float进行精确计算
    error = pid->setpoint - current_value;

    // 2. 比例项
    p_out = pid->Kp * error;

    // 3. 积分项 (带抗饱和)
    pid->integral += error;
    if (pid->integral > pid->integral_max) pid->integral = pid->integral_max;
    else if (pid->integral < -pid->integral_max) pid->integral = -pid->integral_max;
    i_out = pid->Ki * pid->integral;

    // 4. 微分项
    d_out = pid->Kd * (error - pid->last_error);

    // 5. 计算总输出 (float类型)
    output_float = p_out + i_out + d_out;

    // 6. 输出限幅
    if (output_float > pid->output_max) output_float = pid->output_max;
    else if (output_float < pid->output_min) output_float = pid->output_min;

    // 7. 更新 "上一次误差"
    pid->last_error = error;

    // 8. 将最终结果转换为int并返回
    return output_float;
}

int pwm_output_left,pwm_output_right;
/**
 * @brief  执行速度闭环控制 (修改版)
 */
/**
 * @brief  执行速度闭环控制 (已集成死区补偿)
 */
void Speed_Control_Loop(void)
{
    // 1. 计算PID控制器的输出 (这部分保持不变)
    pwm_output_left = PID_Calculate(&g_pid_speed_left, g_left_speed);
    pwm_output_right = PID_Calculate(&g_pid_speed_right, g_right_speed);

    // --- 2. 根据PID输出，结合死区补偿来控制电机 ---

    // ----- 左轮控制 -----
    if (pwm_output_left > 0) // 如果PID想让车轮正转
    {
        // 在PID计算出的值的基础上，加上一个“基础启动动力”
        Motor_Control(MOTOR_LEFT, MOTOR_FORWARD, (uint16_t)(pwm_output_left + MOTOR_DEAD_ZONE));
    }
    else if (pwm_output_left < 0) // 如果PID想让车轮反转
    {
        // 反转同样需要克服死区
        Motor_Control(MOTOR_LEFT, MOTOR_BACKWARD, (uint16_t)(-pwm_output_left + MOTOR_DEAD_ZONE));
    }
    else // 如果PID输出为0
    {
        // 让电机完全停止（或进入刹车状态，取决于你的Motor_Control函数实现）
        Motor_Control(MOTOR_LEFT, MOTOR_STOP, 0);
    }

    // ----- 右轮控制 (逻辑与左轮完全相同) -----
    if (pwm_output_right > 0)
    {
        Motor_Control(MOTOR_RIGHT, MOTOR_FORWARD, (uint16_t)(pwm_output_right + MOTOR_DEAD_ZONE));
    }
    else if (pwm_output_right < 0)
    {
        Motor_Control(MOTOR_RIGHT, MOTOR_BACKWARD, (uint16_t)(-pwm_output_right + MOTOR_DEAD_ZONE));
    }
    else
    {
        Motor_Control(MOTOR_RIGHT, MOTOR_STOP, 0);
    }
}

/**
 * @brief  新增：角度-速度串级控制循环
 * @param  angle_current 当前测量的角度
 * @param  base_speed    基础前进/后退速度
 */
void Angle_Speed_Cascade_Control(float angle_current, float base_speed)
{
    // 1. --- 外环：角度环PID计算 ---
    // 计算出一个转向修正量。这个值代表了速度差
    int turn_output = PID_Calculate(&g_pid_angle, angle_current);

    // 2. --- 设定内环（速度环）的目标值 ---
    // 通过将基础速度与转向修正量结合，来为左右轮设定不同的目标速度
    g_pid_speed_left.setpoint  = base_speed - turn_output;
    g_pid_speed_right.setpoint = base_speed + turn_output;

    // 3. --- 内环：调用速度环PID ---
    // 速度环会根据新的目标值，自动计算PWM并驱动电机
    Speed_Control_Loop();
}