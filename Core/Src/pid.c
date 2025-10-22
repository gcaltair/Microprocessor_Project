/* pid.c (已修正，集成了dt) */
#include <math.h>
#include "system.h" // 确保包含了您的全局变量和函数声明
#include "pid.h"    // 确保包含了pid.h，里面会有新的函数声明
#define SPEED_ERROR_DEADBAND 0.01f
#define ANGLE_CONTROL_DEADBAND 2.0f
#define MOTOR_DEAD_ZONE 1200

/* 定义PID控制器实例 (这些保持不变) */
volatile PID_Controller g_pid_speed_left;
volatile PID_Controller g_pid_speed_right;
volatile PID_Controller g_pid_angle;
volatile float base_car_speed = 0.0f;
volatile int pwm_output_left, pwm_output_right;


/**
 * @brief  初始化PID控制器 (增加更健壮的积分抗饱和设置)
 */
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

    // 【修改】: 设置更合理的积分上限，防止积分饱和 (Integral Windup)
    // 我们限制的是 integral 本身，而不是 i_out。
    // 一个简单有效的方法是，假设我们不希望积分项的输出超过总输出的一半。
    if (pid->Ki > 0.0001f) {
        pid->integral_max = (pid->output_max * 0.5f) / pid->Ki;
    } else {
        pid->integral_max = 0.0f;
    }
}


/**
 * @brief  【步骤一】: 修改最底层的PID计算函数
 * @param  pid: 指向PID_Controller结构体的指针
 * @param  current_value: 当前的测量值
 * @param  dt: 控制周期 (单位：秒) <-- 新增参数
 * @retval 计算得到的PID输出值
 */
float PID_Calculate(PID_Controller *pid, float current_value, float dt)
{
    float error, p_out, i_out, d_out, output_float, derivative;

    // 安全检查，防止dt为0导致程序崩溃
    if (dt <= 0.00001f) {
        return 0;
    }

    error = pid->setpoint - current_value;
    p_out = pid->Kp * error;

    pid->integral += error * dt;

    // 积分抗饱和 (限制 integral 累积值本身)
    if (pid->integral > pid->integral_max) {
        pid->integral = pid->integral_max;
    } else if (pid->integral < -pid->integral_max) {
        pid->integral = -pid->integral_max;
    }

    i_out = pid->Ki * pid->integral;

    // 4. 微分项 (【核心修改】: 除以dt，计算真正的变化率)
    derivative = (error - pid->last_error) / dt;
    d_out = pid->Kd * derivative;

    // 5. 计算总输出 (不变)
    output_float = p_out + i_out + d_out;

    // 6. 输出限幅 (不变)
    if (output_float > pid->output_max) {
        output_float = pid->output_max;
    } else if (output_float < pid->output_min) {
        output_float = pid->output_min;
    }

    // 7. 更新 "上一次误差" (不变)
    pid->last_error = error;

    return output_float;
}


/**
 * @brief  【步骤二】: 修改速度控制循环，传递dt
 * @param  dt: 控制周期 (单位：秒) <-- 新增参数
 */
void Speed_Control_Loop(float dt)
{
    // 1. 像往常一样计算PID输出
    int raw_pwm_left = (int)PID_Calculate(&g_pid_speed_left, g_left_speed, dt);
    int raw_pwm_right = (int)PID_Calculate(&g_pid_speed_right, g_right_speed, dt);

    // if (raw_pwm_left<800) pwm_output_left = 0;
    // else pwm_output_left=raw_pwm_left;
    // if (raw_pwm_right<800) pwm_output_right = 0;
    // else  pwm_output_right=raw_pwm_right;

    pwm_output_left=raw_pwm_left;
    pwm_output_right=raw_pwm_right;

    if (pwm_output_left > 0) {
        Motor_Control(MOTOR_LEFT, MOTOR_FORWARD, (uint16_t)(pwm_output_left + MOTOR_DEAD_ZONE));
    } else if (pwm_output_left < 0) {
        Motor_Control(MOTOR_LEFT, MOTOR_BACKWARD, (uint16_t)(-pwm_output_left + MOTOR_DEAD_ZONE));
    } else {
        Motor_Control(MOTOR_LEFT, MOTOR_STOP, 0);
    }

    if (pwm_output_right > 0) {
        Motor_Control(MOTOR_RIGHT, MOTOR_FORWARD, (uint16_t)(pwm_output_right + MOTOR_DEAD_ZONE));
    } else if (pwm_output_right < 0) {
        Motor_Control(MOTOR_RIGHT, MOTOR_BACKWARD, (uint16_t)(-pwm_output_right + MOTOR_DEAD_ZONE));
    } else {
        Motor_Control(MOTOR_RIGHT, MOTOR_STOP, 0);
    }
}

/**
 * @brief  串级控制总函数，传递dt
 * @param  angle_current 当前测量的角度
 * @param  base_speed    基础前进/后退速度
 * @param  dt: 控制周期 (单位：秒) <-- 新增参数
 */
void Angle_Speed_Cascade_Control(float angle_current, float base_speed, float dt)
{
    float turn_output = 0;
    float error = g_pid_angle.setpoint - angle_current;

    if (fabsf(error) > ANGLE_CONTROL_DEADBAND) {
        turn_output = PID_Calculate(&g_pid_angle, angle_current, dt);
    }

    g_pid_speed_left.setpoint  = base_speed - turn_output;
    g_pid_speed_right.setpoint = base_speed + turn_output;

    Speed_Control_Loop(dt);
}