/* pid.c (已修正，集成了dt) */
#include <math.h>
#include "system.h" // 确保包含了您的全局变量和函数声明
#include "pid.h"    // 确保包含了pid.h，里面会有新的函数声明

#define ANGLE_TOLERANCE_FOR_MOVING 3.0f // 角度容差(度): 朝向与目标夹角小于10度时，才开始前进
/* 定义PID控制器实例 (这些保持不变) */
volatile PID_Controller g_pid_speed_left;
volatile PID_Controller g_pid_speed_right;
volatile PID_Controller g_pid_angle;
volatile PID_Controller g_pid_position;

volatile float base_car_speed = 0.0f;
volatile int pwm_output_left, pwm_output_right;

volatile float g_target_x = 0.0f;
volatile float g_target_y = 0.0f;
volatile RelativeMoveState g_relative_move_state = RELATIVE_MOVE_IDLE;

static float s_target_distance = 0.0f;        // 本次任务需要行驶的距离
static float s_initial_x = 0.0f;              // 任务开始时的X坐标
static float s_initial_y = 0.0f;              // 任务开始时的Y坐标
volatile ControlMode g_control_mode = CONTROL_MODE_MANUAL; // 默认启动时为手动模式

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

    if (dt <= 0.00001f) {
        return 0;
    }

    if (pid->setpoint==0) pid->integral = 0.0f;
    error = pid->setpoint - current_value;
    p_out = pid->Kp * error;

    pid->integral += error * dt;
    if (pid->integral > pid->integral_max) {
        pid->integral = pid->integral_max;
    } else if (pid->integral < -pid->integral_max) {
        pid->integral = -pid->integral_max;
    }

    i_out = pid->Ki * pid->integral;
    derivative = (error - pid->last_error) / dt;
    d_out = pid->Kd * derivative;
    output_float = p_out + i_out + d_out;
    if (output_float > pid->output_max) {
        output_float = pid->output_max;
    } else if (output_float < pid->output_min) {
        output_float = pid->output_min;
    }
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
/**
 * @brief 启动一个相对于当前姿态的移动任务
 * @param dx  相对于车头方向的X位移 (前进为正)
 * @param dy  相对于车头方向的Y位移 (左行为正)
 */
void Start_Relative_Move(float dx, float dy)
{
    // 如果当前正在执行任务，则忽略新指令
    if (g_relative_move_state != RELATIVE_MOVE_IDLE) {
        // 可以发送一个 "busy" 的蓝牙回复
        return;
    }

    // 1. 计算需要行驶的总距离
    s_target_distance = sqrtf(dx * dx + dy * dy);

    // 如果距离太小，直接忽略
    if (s_target_distance < 0.01f) { // 1cm
        return;
    }

    // 2. 计算需要转动的相对角度 (单位：度)
    // atan2f(dy, dx) - 注意这里是dy, dx。在机器人坐标系中，通常x是前进，y是左。
    float relative_angle_deg = atan2f(dy, dx) * 180.0f / PI;

    // 3. 【核心】计算任务最终需要达到的世界绝对角度
    // 假设 g_th_continuous 已经是度
    g_pid_angle.setpoint = g_th_continuous + relative_angle_deg;

    // 4. 记录任务开始时的里程计位置
    s_initial_x = g_x;
    s_initial_y = g_y;

    // 5. 启动状态机，进入“转向”状态
    g_relative_move_state = RELATIVE_MOVE_TURNING;
}

// pid.c

/**
 * @brief 在主循环中被调用，用于更新和执行相对移动任务的状态
 * @param dt 控制周期
 */
void Update_Relative_Move_PID(float dt)
{
    float current_angle_deg = g_th_continuous; // 获取当前世界角度(度)

    switch (g_relative_move_state)
    {
        case RELATIVE_MOVE_IDLE:
            // 空闲状态，确保小车停止
            base_car_speed = 0.0f;
            break;

        case RELATIVE_MOVE_TURNING:
        {
            // 状态一：原地转向
            base_car_speed = 0.0f; // 确保是原地转

            float angle_error = g_pid_angle.setpoint - current_angle_deg;
            // 规范化误差
            while (angle_error > 180.0f)  angle_error -= 360.0f;
            while (angle_error < -180.0f) angle_error += 360.0f;

            // 检查转向是否完成
            if (fabsf(angle_error) < ANGLE_TOLERANCE_FOR_MOVING) // 角度容差
            {
                // 转向完成，进入下一个状态：直行
                g_relative_move_state = RELATIVE_MOVE_DRIVING;
            }
            break;
        }

        case RELATIVE_MOVE_DRIVING:
        {
            // 状态二：直线行驶
            // 1. 计算从任务开始点到当前点已经行驶的距离
            float dx_traveled = g_x - s_initial_x;
            float dy_traveled = g_y - s_initial_y;
            float distance_traveled = sqrtf(dx_traveled * dx_traveled + dy_traveled * dy_traveled);

            // 2. 计算剩余距离
            float distance_error = s_target_distance - distance_traveled;

            // 3. 检查直行是否完成
            if (distance_error < POSITION_REACHED_THRESHOLD) // 距离容差
            {
                // 直行完成，任务结束，返回空闲状态
                g_relative_move_state = RELATIVE_MOVE_IDLE;
                base_car_speed = 0.0f;
                break; // 提前退出，避免执行下面的PID计算
            }

            // 4. 使用位置PID根据剩余距离计算速度
            g_pid_position.setpoint = 0.0f;
            base_car_speed = PID_Calculate(&g_pid_position, -distance_error, dt);

            // 速度限幅
            if (base_car_speed > MAX_BASE_SPEED) base_car_speed = MAX_BASE_SPEED;
            if (base_car_speed < MIN_BASE_SPEED) base_car_speed = MIN_BASE_SPEED;
            break;
        }
    }

    // 无论在哪个状态，都持续调用底层的角度-速度控制器来执行
    Angle_Speed_Cascade_Control(current_angle_deg, base_car_speed, dt);
}


/**
 * @brief  【大脑】位置闭环控制器
 * @param  target_x 目标点的X坐标 (米)
 * @param  target_y 目标点的Y坐标 (米)
 * @param  dt       控制周期 (秒)
 */
// void Position_Control_Loop(float target_x, float target_y, float dt)
// {
//     float error_x, error_y, distance_error;
//
//     // --- 1. 计算误差 ---
//     error_x = target_x - g_x;
//     error_y = target_y - g_y;
//     distance_error = sqrtf(error_x * error_x + error_y * error_y);
//
//     // --- 2. 判断是否已到达目标点 ---
//     if (distance_error < POSITION_REACHED_THRESHOLD)
//     {
//         // 已到达, 命令小车完全停止
//         base_car_speed = 0.0f;
//
//         // 保持最终到达时的姿态，而不是当前姿态
//         float final_angle_deg = g_pid_angle.setpoint; // 获取最后一次的目标角度
//         Angle_Speed_Cascade_Control(final_angle_deg, base_car_speed, dt);
//         return;
//     }
//
//     float target_angle_rad = atan2f(error_y, error_x);
//     float target_angle_deg = target_angle_rad * 180.0f / PI;
//     g_pid_angle.setpoint = g_th_continuous+delta_angle_rad;
//
//     float angle_error = g_pid_angle.setpoint - g_th_continuous;
//
//     while (angle_error > 180.0f)  angle_error -= 360.0f;
//     while (angle_error < -180.0f) angle_error += 360.0f;
//
//     if (fabsf(angle_error) > ANGLE_TOLERANCE_FOR_MOVING)
//     {
//         base_car_speed = 0.0f;
//     }
//     else
//     {
//         g_pid_position.setpoint = 0.0f;
//         base_car_speed = PID_Calculate(&g_pid_position, -distance_error, dt);
//
//         if (base_car_speed > MAX_BASE_SPEED) base_car_speed = MAX_BASE_SPEED;
//         if (base_car_speed < MIN_BASE_SPEED) base_car_speed = MIN_BASE_SPEED;
//     }
//
//     Angle_Speed_Cascade_Control(g_th_continuous, base_car_speed, dt);
// }