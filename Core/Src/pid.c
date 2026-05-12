#include <math.h>

#include "../Inc/control_logic.h"
#include "../Inc/localization_task.h"
#include "freertos_app.h"
#include "pid.h"
#include "system.h"

#define ANGLE_TOLERANCE_FOR_MOVING  3.0f
#define TURN_OUTPUT_LIMIT_MOVING    0.10f
#define TURN_OUTPUT_LIMIT_INPLACE   0.08f

volatile PID_Controller g_pid_speed_left;
volatile PID_Controller g_pid_speed_right;
volatile PID_Controller g_pid_angle;
volatile PID_Controller g_pid_position;

volatile float base_car_speed = 0.0f;
volatile int pwm_output_left = 0;
volatile int pwm_output_right = 0;

volatile float g_target_x = 0.0f;
volatile float g_target_y = 0.0f;
volatile RelativeMoveState g_relative_move_state = RELATIVE_MOVE_IDLE;
volatile ControlMode g_control_mode = CONTROL_MODE_MANUAL;
volatile float g_relative_move_target_distance_m = 0.0f;
volatile float g_relative_move_progress_m = 0.0f;
volatile float g_relative_move_remaining_m = 0.0f;

static float s_target_distance = 0.0f;
static float s_initial_x = 0.0f;
static float s_initial_y = 0.0f;
static float s_drive_direction = 1.0f;
static float s_drive_axis_x = 1.0f;
static float s_drive_axis_y = 0.0f;

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
    output_float = p_out + i_out + d_out;

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

static void pid_get_odometry_pose_snapshot(SlamPose2D_t *pose)
{
    if (pose == NULL) {
        return;
    }

    Odometry_GetPoseSnapshot(pose);
    if (pose->timestamp_ms == 0U) {
        pose->x_m = g_x;
        pose->y_m = g_y;
        pose->theta_deg = g_th_continuous;
        pose->timestamp_ms = HAL_GetTick();
    }
}

static void pid_get_control_pose_snapshot(SlamPose2D_t *pose)
{
    if (pose == NULL) {
        return;
    }

    LocalizationTask_GetControlPoseSnapshot(pose);
    if (pose->timestamp_ms != 0U) {
        return;
    }

    pid_get_odometry_pose_snapshot(pose);
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

float PID_Calculate(PID_Controller *pid, float current_value, float dt)
{
    float error;
    float p_out;
    float i_out;
    float d_out;
    float output_float;
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
    output_float = p_out + i_out + d_out;

    if (output_float > pid->output_max) {
        output_float = pid->output_max;
    } else if (output_float < pid->output_min) {
        output_float = pid->output_min;
    }

    pid->last_error = error;

    return output_float;
}

void Speed_Control_Loop(float dt)
{
    int raw_pwm_left = (int)PID_Calculate((PID_Controller *)&g_pid_speed_left, g_left_speed, dt);
    int raw_pwm_right = (int)PID_Calculate((PID_Controller *)&g_pid_speed_right, g_right_speed, dt);

    pwm_output_left = raw_pwm_left;
    pwm_output_right = raw_pwm_right;

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

void Angle_Speed_Cascade_Control(float angle_current, float base_speed, float dt)
{
    float turn_output = 0.0f;
    float error = pid_normalize_angle_deg(g_pid_angle.setpoint - angle_current);
    float turn_limit = (fabsf(base_speed) > 0.001f) ? TURN_OUTPUT_LIMIT_MOVING : TURN_OUTPUT_LIMIT_INPLACE;

    if (fabsf(error) > ANGLE_CONTROL_DEADBAND) {
        turn_output = pid_calculate_from_error((PID_Controller *)&g_pid_angle, error, dt);
        turn_output = pid_clamp_float(turn_output, -turn_limit, turn_limit);
    } else {
        g_pid_angle.integral = 0.0f;
        g_pid_angle.last_error = 0.0f;
    }

    g_pid_speed_left.setpoint = base_speed - turn_output;
    g_pid_speed_right.setpoint = base_speed + turn_output;

    Speed_Control_Loop(dt);
}

void Control_SetManualCommand(float command_base_speed, float angle_setpoint)
{
    lock_control_and_pid();

    g_relative_move_state = RELATIVE_MOVE_IDLE;
    g_control_mode = CONTROL_MODE_MANUAL;
    base_car_speed = command_base_speed;
    g_pid_angle.setpoint = ControlLogic_WrapAngleDeg(angle_setpoint);
    g_relative_move_target_distance_m = 0.0f;
    g_relative_move_progress_m = 0.0f;
    g_relative_move_remaining_m = 0.0f;
    g_target_x = 0.0f;
    g_target_y = 0.0f;

    unlock_pid_and_control();
}

void Control_SetManualDrive(float command_base_speed)
{
    SlamPose2D_t pose;

    lock_odom_control_and_pid();
    pid_get_control_pose_snapshot(&pose);

    g_relative_move_state = RELATIVE_MOVE_IDLE;
    g_control_mode = CONTROL_MODE_MANUAL;
    base_car_speed = command_base_speed;
    g_pid_angle.setpoint = pose.theta_deg;
    g_relative_move_target_distance_m = 0.0f;
    g_relative_move_progress_m = 0.0f;
    g_relative_move_remaining_m = 0.0f;
    g_target_x = 0.0f;
    g_target_y = 0.0f;

    unlock_pid_control_and_odom();
}

void Control_SetRelativeTurn(float delta_angle)
{
    SlamPose2D_t pose;

    lock_odom_control_and_pid();
    pid_get_control_pose_snapshot(&pose);

    g_relative_move_state = RELATIVE_MOVE_IDLE;
    g_control_mode = CONTROL_MODE_MANUAL;
    base_car_speed = 0.0f;
    g_pid_angle.setpoint = ControlLogic_ResolveAbsoluteSetpointFromCurrentHeading(pose.theta_deg,
                                                                                  delta_angle);
    g_relative_move_target_distance_m = 0.0f;
    g_relative_move_progress_m = 0.0f;
    g_relative_move_remaining_m = 0.0f;
    g_target_x = 0.0f;
    g_target_y = 0.0f;

    unlock_pid_control_and_odom();
}

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

void Control_StopCommand(void)
{
    SlamPose2D_t pose;

    lock_odom_control_and_pid();
    pid_get_control_pose_snapshot(&pose);

    g_relative_move_state = RELATIVE_MOVE_IDLE;
    g_control_mode = CONTROL_MODE_MANUAL;
    base_car_speed = 0.0f;
    s_drive_direction = 1.0f;
    pid_set_drive_axis_from_heading_deg(pose.theta_deg);
    g_pid_angle.setpoint = pose.theta_deg;
    g_relative_move_target_distance_m = 0.0f;
    g_relative_move_progress_m = 0.0f;
    g_relative_move_remaining_m = 0.0f;
    g_target_x = 0.0f;
    g_target_y = 0.0f;

    unlock_pid_control_and_odom();
}

void Start_Relative_Move(float dx, float dy)
{
    SlamPose2D_t pose;
    float target_heading_deg;

    lock_odom_control_and_pid();
    pid_get_odometry_pose_snapshot(&pose);

    if (g_relative_move_state != RELATIVE_MOVE_IDLE) {
        unlock_pid_control_and_odom();
        return;
    }

    s_target_distance = sqrtf(dx * dx + dy * dy);
    if (s_target_distance < 0.01f) {
        unlock_pid_control_and_odom();
        return;
    }

    g_target_x = dx;
    g_target_y = dy;
    g_relative_move_target_distance_m = s_target_distance;
    g_relative_move_progress_m = 0.0f;
    g_relative_move_remaining_m = s_target_distance;

    target_heading_deg = atan2f(dy, dx) * 180.0f / PI;
    s_drive_direction = 1.0f;
    if (target_heading_deg > 90.0f) {
        target_heading_deg -= 180.0f;
        s_drive_direction = -1.0f;
    } else if (target_heading_deg < -90.0f) {
        target_heading_deg += 180.0f;
        s_drive_direction = -1.0f;
    }

    g_pid_angle.setpoint = pose.theta_deg + target_heading_deg;
    s_initial_x = pose.x_m;
    s_initial_y = pose.y_m;
    pid_set_drive_axis_from_heading_deg(g_pid_angle.setpoint);
    g_pid_position.integral = 0.0f;
    g_pid_position.last_error = 0.0f;
    base_car_speed = 0.0f;
    g_control_mode = CONTROL_MODE_POSITION;
    g_relative_move_state = RELATIVE_MOVE_TURNING;

    unlock_pid_control_and_odom();
}

void Update_Relative_Move_PID(float dt, const SlamPose2D_t *pose)
{
    float current_angle_deg;
    float current_x_m;
    float current_y_m;

    if (pose != NULL) {
        current_angle_deg = pose->theta_deg;
    } else {
        current_angle_deg = g_th_continuous;
    }

    current_x_m = g_x;
    current_y_m = g_y;

    switch (g_relative_move_state)
    {
        case RELATIVE_MOVE_IDLE:
            base_car_speed = 0.0f;
            g_relative_move_progress_m = 0.0f;
            g_relative_move_remaining_m = 0.0f;
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
                g_relative_move_progress_m = 0.0f;
                g_relative_move_remaining_m = s_target_distance;
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

            if (distance_progress < 0.0f) {
                distance_progress = 0.0f;
            }

            /*
             * Use commanded-path progress instead of start-to-current Euclidean distance.
             * When the robot drives while trimming heading, arc motion makes straight-line
             * displacement smaller than real path progress and causes consistent overshoot.
             */
            distance_error = s_target_distance - distance_progress;
            g_relative_move_progress_m = distance_progress;
            g_relative_move_remaining_m = (distance_error > 0.0f) ? distance_error : 0.0f;

            if (distance_error < POSITION_REACHED_THRESHOLD) {
                g_relative_move_state = RELATIVE_MOVE_IDLE;
                g_control_mode = CONTROL_MODE_MANUAL;
                base_car_speed = 0.0f;
                s_drive_direction = 1.0f;
                g_pid_position.integral = 0.0f;
                g_pid_position.last_error = 0.0f;
                pid_set_drive_axis_from_heading_deg(current_angle_deg);
                g_pid_angle.setpoint = current_angle_deg;
                g_relative_move_target_distance_m = 0.0f;
                g_relative_move_progress_m = 0.0f;
                g_relative_move_remaining_m = 0.0f;
                g_target_x = 0.0f;
                g_target_y = 0.0f;
                break;
            }

            g_pid_position.setpoint = 0.0f;
            base_car_speed = s_drive_direction * PID_Calculate((PID_Controller *)&g_pid_position, -distance_error, dt);

            if (base_car_speed > MAX_BASE_SPEED) {
                base_car_speed = MAX_BASE_SPEED;
            }

            if ((base_car_speed > 0.0f) && (base_car_speed < MIN_BASE_SPEED)) {
                base_car_speed = MIN_BASE_SPEED;
            } else if ((base_car_speed < 0.0f) && (base_car_speed > -MIN_BASE_SPEED)) {
                base_car_speed = -MIN_BASE_SPEED;
            }
            break;
        }
    }

    Angle_Speed_Cascade_Control(current_angle_deg, base_car_speed, dt);
}
