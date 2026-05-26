#include <math.h>

#include "../Inc/control_logic.h"
#include "freertos_app.h"
#include "pid.h"
#include "system.h"

/*
 * 控制链路总览：
 *
 * 1. StartControlTask 每 10ms 更新传感器、编码器速度和里程计位姿，然后按 g_control_mode 调用本文件。
 * 2. 普通手动模式：base_car_speed 给出前进速度，g_pid_angle.setpoint 给出目标航向，
 *    Angle_Speed_Cascade_Control() 用角度环算出左右轮差速修正。
 * 3. 相对位移模式：Start_Relative_Move(dx, dy) 先把位移命令转换成目标航向和目标距离；
 *    Update_Relative_Move_PID() 先原地转到目标航向，再由位置环算出 base_car_speed。
 * 4. 最后所有非轮速测试模式都会进入 Angle_Speed_Cascade_Control()：
 *    目标航向 - 当前航向 -> turn_output -> 左右轮速度 setpoint。
 * 5. Speed_Control_Loop() 是最内层速度环：
 *    左右轮速度 setpoint - 编码器实测速度 -> PWM -> Motor_Control()。
 */
#define ANGLE_TOLERANCE_FOR_MOVING  0.8f
#define ANGLE_CONTROL_DEADBAND_ENTER_DEG  0.3f
#define ANGLE_CONTROL_DEADBAND_EXIT_DEG   0.5f
#define TURN_OUTPUT_LIMIT_MOVING    0.5f
#define TURN_OUTPUT_LIMIT_INPLACE   0.6f
#define TURN_OUTPUT_MOVING_BASE_RATIO     1.00f

volatile PID_Controller g_pid_speed_left;
volatile PID_Controller g_pid_speed_right;
volatile PID_Controller g_pid_angle;
volatile PID_Controller g_pid_position;

volatile float base_car_speed = 0.0f;

volatile RelativeMoveState g_relative_move_state = RELATIVE_MOVE_IDLE;
volatile ControlMode g_control_mode = CONTROL_MODE_MANUAL;

/* 当前相对位移命令的目标路程，单位 m，由 Start_Relative_Move(dx, dy) 根据位移向量长度计算。 */
static float s_target_distance = 0.0f;

/* 进入直线行驶阶段时的里程计起点，用来计算本段相对位移已经走了多少。 */
static float s_initial_x = 0.0f;
static float s_initial_y = 0.0f;

/*
 * 行驶方向标志：1 表示向前走，-1 表示倒车走。
 * Start_Relative_Move() 始终使用 1，让目标在后方时先掉头再前进；
 * Start_Relative_Drive() 仍可通过负距离单独启动倒车直行。
 */
static float s_drive_direction = 1.0f;

/*
 * 当前运动段的行驶轴单位向量，来自目标航向角的 cos/sin。
 * DRIVING 阶段把当前位置变化投影到这条轴上，得到沿命令方向的进度。
 */
static float s_drive_axis_x = 1.0f;
static float s_drive_axis_y = 0.0f;

/*
 * 角度环死区迟滞状态。
 * 误差超过退出阈值时开始角度修正，误差回到进入阈值内时关闭修正，避免小误差附近反复抖动。
 */
static uint8_t s_angle_control_active = 0U;
static ControlDebugSnapshot_t s_control_debug_snapshot;
static float s_speed_limit_mps = MAX_BASE_SPEED;

static float pid_normalize_angle_deg(float angle_deg)
{
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }

    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }

    return angle_deg;
}

static float pid_calculate_from_error(PID_Controller *pid, float error, float dt)
{
    float p_out;
    float i_out;
    float d_out;
    float output_float;
    float unclamped_output;
    float derivative;

    if (dt <= 0.00001f) {
        return 0.0f;
    }

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
    unclamped_output = p_out + i_out + d_out;
    output_float = unclamped_output;

    if (output_float > pid->output_max) {
        output_float = pid->output_max;
    } else if (output_float < pid->output_min) {
        output_float = pid->output_min;
    }

    pid->last_error = error;

    return output_float;
}

static float pid_clamp_float(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }

    if (value > max_value) {
        return max_value;
    }

    return value;
}

static void pid_set_drive_axis_from_heading_deg(float heading_deg)
{
    float heading_rad = heading_deg * PI / 180.0f;

    s_drive_axis_x = cosf(heading_rad);
    s_drive_axis_y = sinf(heading_rad);
}

static void lock_control_and_pid(void)
{
    if (g_controlMutex != NULL) {
        (void)osMutexAcquire(g_controlMutex, osWaitForever);
    }

    if (g_pidMutex != NULL) {
        (void)osMutexAcquire(g_pidMutex, osWaitForever);
    }
}

static void unlock_pid_and_control(void)
{
    if (g_pidMutex != NULL) {
        (void)osMutexRelease(g_pidMutex);
    }

    if (g_controlMutex != NULL) {
        (void)osMutexRelease(g_controlMutex);
    }
}

static void lock_odom_control_and_pid(void)
{
    if (g_odomMutex != NULL) {
        (void)osMutexAcquire(g_odomMutex, osWaitForever);
    }

    lock_control_and_pid();
}

static void unlock_pid_control_and_odom(void)
{
    unlock_pid_and_control();

    if (g_odomMutex != NULL) {
        (void)osMutexRelease(g_odomMutex);
    }
}

static PID_Controller *pid_select_controller(char loop_id)
{
    switch (loop_id) {
        case 'A':
        case 'a':
            return (PID_Controller *)&g_pid_angle;
        case 'L':
        case 'l':
            return (PID_Controller *)&g_pid_speed_left;
        case 'R':
        case 'r':
            return (PID_Controller *)&g_pid_speed_right;
        case 'P':
        case 'p':
            return (PID_Controller *)&g_pid_position;
        default:
            return NULL;
    }
}

/*
 * 初始化一个 PID 控制器。
 *
 * setpoint 初始化为 0，积分和上一次误差清零；如果 Ki 有效，则按输出上限给积分项设置限幅，
 * 避免长时间误差累计后输出严重饱和。
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

    if (pid->Ki > 0.0001f) {
        pid->integral_max = (pid->output_max * 0.5f) / pid->Ki;
    } else {
        pid->integral_max = 0.0f;
    }
}

/*
 * 通用 PID 计算函数，适合速度环和位置环这种“setpoint - current_value”的普通误差形式。
 *
 * 速度环中 current_value 是编码器测得的轮速；位置环中 current_value 传入 -distance_error，
 * 因为位置环 setpoint 固定为 0，用这种写法可以让距离误差越大，输出的基础速度越大。
 */
float PID_Calculate(PID_Controller *pid, float current_value, float dt)
{
    float error;
    float p_out;
    float i_out;
    float d_out;
    float output_float;
    float unclamped_output;
    float derivative;

    if (dt <= 0.00001f) {
        return 0.0f;
    }

    if (pid->setpoint == 0.0f) {
        pid->integral = 0.0f;
    }

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
    unclamped_output = p_out + i_out + d_out;
    output_float = unclamped_output;

    if (output_float > pid->output_max) {
        output_float = pid->output_max;
    } else if (output_float < pid->output_min) {
        output_float = pid->output_min;
    }

    pid->last_error = error;

    return output_float;
}

/*
 * 最内层左右轮速度闭环。
 *
 * 输入来自 g_pid_speed_left/right.setpoint，反馈来自 encoder.c 更新的 g_left_speed/g_right_speed。
 * PID 输出先转换成带符号 PWM，再叠加电机死区补偿，最后调用 Motor_Control() 设置方向和占空比。
 */
void Speed_Control_Loop(float dt)
{
    int raw_pwm_left = (int)PID_Calculate((PID_Controller *)&g_pid_speed_left, g_left_speed, dt);
    int raw_pwm_right = (int)PID_Calculate((PID_Controller *)&g_pid_speed_right, g_right_speed, dt);
    int applied_pwm_left = 0;
    int applied_pwm_right = 0;
    uint8_t direction_left = MOTOR_STOP;
    uint8_t direction_right = MOTOR_STOP;

    if (raw_pwm_left > 0) {
        direction_left = MOTOR_FORWARD;
        applied_pwm_left = raw_pwm_left + MOTOR_DEAD_ZONE;
        if (applied_pwm_left > 10000) {
            applied_pwm_left = 10000;
        }
        Motor_Control(MOTOR_LEFT, direction_left, (uint16_t)applied_pwm_left);
    } else if (raw_pwm_left < 0) {
        direction_left = MOTOR_BACKWARD;
        applied_pwm_left = -raw_pwm_left + MOTOR_DEAD_ZONE;
        if (applied_pwm_left > 10000) {
            applied_pwm_left = 10000;
        }
        Motor_Control(MOTOR_LEFT, direction_left, (uint16_t)applied_pwm_left);
    } else {
        Motor_Control(MOTOR_LEFT, MOTOR_STOP, 0);
    }

    if (raw_pwm_right > 0) {
        direction_right = MOTOR_FORWARD;
        applied_pwm_right = raw_pwm_right + MOTOR_DEAD_ZONE;
        if (applied_pwm_right > 10000) {
            applied_pwm_right = 10000;
        }
        Motor_Control(MOTOR_RIGHT, direction_right, (uint16_t)applied_pwm_right);
    } else if (raw_pwm_right < 0) {
        direction_right = MOTOR_BACKWARD;
        applied_pwm_right = -raw_pwm_right + MOTOR_DEAD_ZONE;
        if (applied_pwm_right > 10000) {
            applied_pwm_right = 10000;
        }
        Motor_Control(MOTOR_RIGHT, direction_right, (uint16_t)applied_pwm_right);
    } else {
        Motor_Control(MOTOR_RIGHT, MOTOR_STOP, 0);
    }

    s_control_debug_snapshot.left_direction = direction_left;
    s_control_debug_snapshot.right_direction = direction_right;
    s_control_debug_snapshot.left_pwm = (uint16_t)applied_pwm_left;
    s_control_debug_snapshot.right_pwm = (uint16_t)applied_pwm_right;
    s_control_debug_snapshot.left_speed_pid_output = (int16_t)raw_pwm_left;
    s_control_debug_snapshot.right_speed_pid_output = (int16_t)raw_pwm_right;
    s_control_debug_snapshot.left_speed_feedback_mps = g_left_speed;
    s_control_debug_snapshot.right_speed_feedback_mps = g_right_speed;
}

/*
 * 角度环 + 速度环串级控制。
 *
 * 外层角度环：g_pid_angle.setpoint - angle_current 得到航向误差，输出 turn_output。
 * 差速混合：左轮目标速度 = base_speed - turn_output，右轮目标速度 = base_speed + turn_output。
 * 内层速度环：Speed_Control_Loop() 继续把左右轮目标速度闭环到 PWM。
 */
void Angle_Speed_Cascade_Control(float angle_current, float base_speed, float dt)
{
    float turn_output = 0.0f;
    float raw_turn_output = 0.0f;
    float error = pid_normalize_angle_deg(g_pid_angle.setpoint - angle_current);
    float turn_limit;
    float abs_base_speed;
    float abs_error = fabsf(error);

    base_speed = pid_clamp_float(base_speed, -s_speed_limit_mps, s_speed_limit_mps);
    base_car_speed = base_speed;
    abs_base_speed = fabsf(base_speed);
    turn_limit = (abs_base_speed > 0.001f) ? TURN_OUTPUT_LIMIT_MOVING : TURN_OUTPUT_LIMIT_INPLACE;

    if (abs_base_speed > 0.001f) {
        float moving_limit = abs_base_speed * TURN_OUTPUT_MOVING_BASE_RATIO;
        if (turn_limit > moving_limit) {
            turn_limit = moving_limit;
        }
    }

    /*
     * 角度环使用进入/退出两个阈值形成迟滞：
     * 误差较小时关闭角度修正，误差再次变大后才重新打开，减少直行时的小幅左右抖动。
     */
    if (s_angle_control_active != 0U) {
        if (abs_error <= ANGLE_CONTROL_DEADBAND_ENTER_DEG) {
            s_angle_control_active = 0U;
        }
    } else if (abs_error >=  ANGLE_CONTROL_DEADBAND_EXIT_DEG) {
        s_angle_control_active = 1U;
    }

    if (s_angle_control_active != 0U) {
        raw_turn_output = pid_calculate_from_error((PID_Controller *)&g_pid_angle, error, dt);
        turn_output = pid_clamp_float(raw_turn_output, -turn_limit, turn_limit);
    } else {
        g_pid_angle.integral = 0.0f;
        g_pid_angle.last_error = 0.0f;
    }

    g_pid_speed_left.setpoint = base_speed - turn_output;
    g_pid_speed_right.setpoint = base_speed + turn_output;
    s_control_debug_snapshot.control_mode = (uint8_t)g_control_mode;
    s_control_debug_snapshot.relative_move_state = (uint8_t)g_relative_move_state;
    s_control_debug_snapshot.angle_pid_output_mps = turn_output;
    s_control_debug_snapshot.angle_error_deg = error;
    s_control_debug_snapshot.base_speed_mps = base_speed;
    s_control_debug_snapshot.left_speed_setpoint_mps = g_pid_speed_left.setpoint;
    s_control_debug_snapshot.right_speed_setpoint_mps = g_pid_speed_right.setpoint;

    Speed_Control_Loop(dt);
}

/*
 * 轮速测试模式。
 *
 * 该模式绕过角度环和位置环，直接使用已经写入 g_pid_speed_left/right.setpoint 的左右轮目标速度，
 * 用来单独验证编码器测速、速度 PID、电机方向和 PWM 输出是否正常。
 */
void Control_UpdateWheelSpeedTest(float dt)
{
    float left_setpoint = pid_clamp_float(g_pid_speed_left.setpoint, -s_speed_limit_mps, s_speed_limit_mps);
    float right_setpoint = pid_clamp_float(g_pid_speed_right.setpoint, -s_speed_limit_mps, s_speed_limit_mps);

    g_pid_speed_left.setpoint = left_setpoint;
    g_pid_speed_right.setpoint = right_setpoint;

    base_car_speed = (left_setpoint + right_setpoint) * 0.5f;
    s_control_debug_snapshot.control_mode = (uint8_t)g_control_mode;
    s_control_debug_snapshot.relative_move_state = (uint8_t)g_relative_move_state;
    s_control_debug_snapshot.position_pid_output_mps = 0.0f;
    s_control_debug_snapshot.position_error_m = 0.0f;
    s_control_debug_snapshot.angle_pid_output_mps = 0.0f;
    s_control_debug_snapshot.angle_error_deg = 0.0f;
    s_control_debug_snapshot.base_speed_mps = base_car_speed;
    s_control_debug_snapshot.left_speed_setpoint_mps = left_setpoint;
    s_control_debug_snapshot.right_speed_setpoint_mps = right_setpoint;

    Speed_Control_Loop(dt);
}

void Control_GetDebugSnapshot(ControlDebugSnapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return;
    }

    if (g_pidMutex != NULL) {
        (void)osMutexAcquire(g_pidMutex, osWaitForever);
    }

    *snapshot = s_control_debug_snapshot;

    if (g_pidMutex != NULL) {
        (void)osMutexRelease(g_pidMutex);
    }
}

uint8_t Control_GetPidTunings(char loop_id, float *kp, float *ki, float *kd)
{
    PID_Controller *pid = pid_select_controller(loop_id);

    if ((pid == NULL) || (kp == NULL) || (ki == NULL) || (kd == NULL)) {
        return 0U;
    }

    if (g_pidMutex != NULL) {
        (void)osMutexAcquire(g_pidMutex, osWaitForever);
    }

    *kp = pid->Kp;
    *ki = pid->Ki;
    *kd = pid->Kd;

    if (g_pidMutex != NULL) {
        (void)osMutexRelease(g_pidMutex);
    }

    return 1U;
}

/* 设置某个 PID 环的 Kp/Ki/Kd，目前用于预留的在线调参入口。 */
uint8_t Control_SetPidTunings(char loop_id, float kp, float ki, float kd)
{
    PID_Controller *pid = pid_select_controller(loop_id);

    if ((pid == NULL) ||
        (isfinite(kp) == 0) ||
        (isfinite(ki) == 0) ||
        (isfinite(kd) == 0) ||
        (kp < 0.0f) ||
        (ki < 0.0f) ||
        (kd < 0.0f)) {
        return 0U;
    }

    if (g_pidMutex != NULL) {
        (void)osMutexAcquire(g_pidMutex, osWaitForever);
    }

    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    if (pid->Ki > 0.0001f) {
        pid->integral_max = (pid->output_max * 0.5f) / pid->Ki;
    } else {
        pid->integral_max = 0.0f;
    }

    if (g_pidMutex != NULL) {
        (void)osMutexRelease(g_pidMutex);
    }

    return 1U;
}

/*
 * 手动速度 + 指定航向模式。
 *
 * 上层直接给基础速度 command_base_speed 和绝对目标角 angle_setpoint；
 * 控制任务随后在普通手动分支中调用 Angle_Speed_Cascade_Control() 保持这个航向。
 */
void Control_SetManualCommand(float command_base_speed, float angle_setpoint)
{
    lock_control_and_pid();

    g_relative_move_state = RELATIVE_MOVE_IDLE;
    g_control_mode = CONTROL_MODE_MANUAL;
    base_car_speed = command_base_speed;
    g_pid_angle.setpoint = ControlLogic_WrapAngleDeg(angle_setpoint);
    s_angle_control_active = 0U;

    unlock_pid_and_control();
}

/* 只修改基础速度，不改变当前角度目标；适合在保持航向的同时调整前进/后退速度。 */
void Control_SetBaseSpeed(float command_base_speed)
{
    if (g_pidMutex != NULL) {
        (void)osMutexAcquire(g_pidMutex, osWaitForever);
    }

    base_car_speed = command_base_speed;

    if (g_pidMutex != NULL) {
        (void)osMutexRelease(g_pidMutex);
    }
}

void Control_SetSpeedLimit(float speed_limit_mps)
{
    if (speed_limit_mps < 0.01f) {
        speed_limit_mps = 0.01f;
    } else if (speed_limit_mps > MAX_BASE_SPEED) {
        speed_limit_mps = MAX_BASE_SPEED;
    }

    if (g_pidMutex != NULL) {
        (void)osMutexAcquire(g_pidMutex, osWaitForever);
    }

    s_speed_limit_mps = speed_limit_mps;

    if (g_pidMutex != NULL) {
        (void)osMutexRelease(g_pidMutex);
    }
}

/*
 * 设置左右轮速度测试目标。
 *
 * 进入 CONTROL_MODE_SPEED_TEST 后，控制任务不再经过角度环和位置环，
 * 而是每周期直接调用 Control_UpdateWheelSpeedTest()。
 */
uint8_t Control_SetWheelSpeedTest(float left_speed_mps, float right_speed_mps)
{
    if ((isfinite(left_speed_mps) == 0) ||
        (isfinite(right_speed_mps) == 0) ||
        (fabsf(left_speed_mps) > SPEED_TEST_MAX_SETPOINT_MPS) ||
        (fabsf(right_speed_mps) > SPEED_TEST_MAX_SETPOINT_MPS)) {
        return 0U;
    }

    lock_control_and_pid();

    g_relative_move_state = RELATIVE_MOVE_IDLE;
    g_control_mode = CONTROL_MODE_SPEED_TEST;
    base_car_speed = (left_speed_mps + right_speed_mps) * 0.5f;
    g_pid_speed_left.setpoint = left_speed_mps;
    g_pid_speed_right.setpoint = right_speed_mps;
    g_pid_speed_left.integral = 0.0f;
    g_pid_speed_left.last_error = 0.0f;
    g_pid_speed_right.integral = 0.0f;
    g_pid_speed_right.last_error = 0.0f;
    g_pid_angle.integral = 0.0f;
    g_pid_angle.last_error = 0.0f;
    g_pid_position.integral = 0.0f;
    g_pid_position.last_error = 0.0f;
    s_angle_control_active = 0U;

    unlock_pid_and_control();

    return 1U;
}
void Start_Relative_Turn(float angle_delta)
{
    lock_odom_control_and_pid();
    if (g_relative_move_state != RELATIVE_MOVE_IDLE) {
        unlock_pid_control_and_odom();
        return;
    }
    s_target_distance = 0.0f;
    g_pid_angle.setpoint += angle_delta;
    pid_set_drive_axis_from_heading_deg(g_pid_angle.setpoint);

    g_pid_position.integral = 0.0f;
    g_pid_position.last_error = 0.0f;
    base_car_speed = 0.0f;
    g_control_mode = CONTROL_MODE_POSITION;
    g_relative_move_state = RELATIVE_MOVE_TURNING;
    s_angle_control_active = 0U;
    unlock_pid_control_and_odom();
}
void Start_Relative_Drive(float distance_m)
{
    SlamPose2D_t pose;

    lock_odom_control_and_pid();
    Odometry_GetPoseSnapshot(&pose);

    if (g_relative_move_state != RELATIVE_MOVE_IDLE) {
        unlock_pid_control_and_odom();
        return;
    }

    s_target_distance = fabsf(distance_m);
    if (s_target_distance < POSITION_REACHED_THRESHOLD) {
        unlock_pid_control_and_odom();
        return;
    }

    s_drive_direction = (distance_m >= 0.0f) ? 1.0f : -1.0f;

    s_initial_x = pose.x_m;
    s_initial_y = pose.y_m;

    g_pid_position.integral = 0.0f;
    g_pid_position.last_error = 0.0f;
    base_car_speed = 0.0f;
    g_control_mode = CONTROL_MODE_POSITION;
    g_relative_move_state = RELATIVE_MOVE_DRIVING; // 直接进入直行状态
    s_angle_control_active = 0U;

    unlock_pid_control_and_odom();
}
/*
 * 启动一次相对位移运动。
 *
 * dx/dy 表示相对位移向量，函数会计算目标距离和目标航向。
 * 相对移动只允许向前走；若目标方向在车身后方，先原地掉头，再向前直行。
 */
void Start_Relative_Move(float dx, float dy)
{
    SlamPose2D_t pose;
    float target_heading_deg;

    lock_odom_control_and_pid();
    Odometry_GetPoseSnapshot(&pose);

    if (g_relative_move_state != RELATIVE_MOVE_IDLE) {
        unlock_pid_control_and_odom();
        return;
    }

    s_target_distance = sqrtf(dx * dx + dy * dy);
    if (s_target_distance < POSITION_REACHED_THRESHOLD) {
        unlock_pid_control_and_odom();
        return;
    }

    target_heading_deg = atan2f(dy, dx) * 180.0f / PI;

    s_drive_direction = 1.0f;
    g_pid_angle.setpoint = target_heading_deg;
    pid_set_drive_axis_from_heading_deg(g_pid_angle.setpoint);

    s_initial_x = pose.x_m;
    s_initial_y = pose.y_m;

    g_pid_position.integral = 0.0f;
    g_pid_position.last_error = 0.0f;
    base_car_speed = 0.0f;
    g_control_mode = CONTROL_MODE_POSITION;
    g_relative_move_state = RELATIVE_MOVE_TURNING;
    s_angle_control_active = 0U;

    unlock_pid_control_and_odom();
}

/*
 * 取消正在执行的相对位移运动。
 *
 * 导航任务在重新规划路径后发现新方向与当前运动方向偏差过大时调用此函数，
 * 立即中断当前 relative_move 并将控制层恢复为空闲状态，为后续下发新运动
 * 腾出条件。若当前不在相对位移模式，则不做任何操作。
 */
void Cancel_Relative_Move(void)
{
    lock_control_and_pid();

    if (g_relative_move_state != RELATIVE_MOVE_IDLE) {
        g_relative_move_state = RELATIVE_MOVE_IDLE;
        g_control_mode = CONTROL_MODE_MANUAL;
        base_car_speed = 0.0f;
        g_pid_position.integral = 0.0f;
        g_pid_position.last_error = 0.0f;
        s_angle_control_active = 0U;
    }

    unlock_pid_and_control();
}

/*
 * 相对位移模式的外层状态机。
 *
 * TURNING：base_car_speed 为 0，只用角度环把车头转到目标航向。
 * DRIVING：根据沿行驶轴的投影进度计算剩余距离，由位置环生成 base_car_speed；
 *          随后仍调用 Angle_Speed_Cascade_Control()，让车辆边走边保持目标航向。
 */
void Update_Relative_Move_PID(float dt, const SlamPose2D_t *pose)
{
    float current_angle_deg;
    float current_x_m;
    float current_y_m;
    SlamPose2D_t local_pose;

    if (pose == NULL) {
        Odometry_GetPoseSnapshot(&local_pose);
        pose = &local_pose;
    }

    current_angle_deg = pose->theta_deg;
    current_x_m = pose->x_m;
    current_y_m = pose->y_m;

    switch (g_relative_move_state)
    {
        case RELATIVE_MOVE_IDLE:
            base_car_speed = 0.0f;
            break;

        case RELATIVE_MOVE_TURNING:
        {
            float angle_error;

            base_car_speed = 0.0f;
            angle_error = pid_normalize_angle_deg(g_pid_angle.setpoint - current_angle_deg);

            if (fabsf(angle_error) < ANGLE_TOLERANCE_FOR_MOVING) {
                s_initial_x = current_x_m;
                s_initial_y = current_y_m;
                pid_set_drive_axis_from_heading_deg(current_angle_deg);
                g_pid_position.integral = 0.0f;
                g_pid_position.last_error = 0.0f;
                g_relative_move_state = RELATIVE_MOVE_DRIVING;
            }
            break;
        }

        case RELATIVE_MOVE_DRIVING:
        {
            float dx_traveled = current_x_m - s_initial_x;
            float dy_traveled = current_y_m - s_initial_y;
            float distance_progress = (dx_traveled * s_drive_axis_x + dy_traveled * s_drive_axis_y) * s_drive_direction;
            float distance_error;
            float raw_base_speed;

            if (distance_progress < 0.0f) {
                distance_progress = 0.0f;
            }
            distance_error = s_target_distance - distance_progress;

            if (distance_error < POSITION_REACHED_THRESHOLD) {
                g_relative_move_state = RELATIVE_MOVE_IDLE;
                g_control_mode = CONTROL_MODE_MANUAL;
                base_car_speed = 0.0f;
                s_drive_direction = 1.0f;
                g_pid_position.integral = 0.0f;
                g_pid_position.last_error = 0.0f;
                s_angle_control_active = 0U;
                break;
            }

            g_pid_position.setpoint = 0.0f;
            raw_base_speed = s_drive_direction * PID_Calculate((PID_Controller *)&g_pid_position, -distance_error, dt);

            base_car_speed = raw_base_speed;

            if (base_car_speed > MAX_BASE_SPEED) {
                base_car_speed = MAX_BASE_SPEED;
            } else if (base_car_speed < -MAX_BASE_SPEED) {
                base_car_speed = -MAX_BASE_SPEED;
            }
            break;
        }
    }

    Angle_Speed_Cascade_Control(current_angle_deg, base_car_speed, dt);
}
